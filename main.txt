// main.c — Projeto Orientado (GY-33 + BH1750 + OLED + WS2812 + Buzzer PWM + LED RGB via Botão A)
// BitDogLab + RP2040 (Pico W)
//
// A matriz WS2812 só é atualizada depois do GY-33 confirmar a cor.
// O LED discreto muda devagar (5s por cor) mas sem travar o loop.

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/bootrom.h"

// ==== OLED (I2C1) ====
#include "ssd1306.h"
#include "font.h"

// ==== BH1750 (I2C0) ====
#include "bh1750_light_sensor.h"

// ==== WS2812 (sua lib) ====
#include "ws2812.h"

// =================== Barramentos ===================
#define I2C_SENS i2c0
#define I2C_SENS_SDA 0
#define I2C_SENS_SCL 1

#define I2C_DISP i2c1
#define I2C_DISP_SDA 14
#define I2C_DISP_SCL 15
#define OLED_ADDR 0x3C

// =================== GY-33 / TCS34725 (I2C0) ===================
#define GY33_ADDR 0x29
#define REG_ENABLE 0x80
#define REG_ATIME 0x81
#define REG_CONTROL 0x8F
#define REG_CDATA 0x94
#define REG_RDATA 0x96
#define REG_GDATA 0x98
#define REG_BDATA 0x9A

// =================== WS2812 ===================
#define WS2812_PIN 7
#define WS2812_COUNT 25

// =================== Buzzer PWM ===================
#define BUZZER_PIN 21

// =================== Botões ===================
#define BTN_A_PIN 5 // alterna modos LED RGB
#define BTN_B_PIN 6 // BOOTSEL

// =================== LED RGB Discreto ===================
#define LED_R_PIN 13
#define LED_G_PIN 11
#define LED_B_PIN 12
#define LED_COMMON_ANODE 0

// =================== Parâmetros ===================
#define LUX_LOW_THRESHOLD 10
// loop principal não bloqueante: pequeno delay por iteração
#define LOOP_DELAY_MS 40
#define BUZZER_ALERT_MS 150
#define BTN_DEBOUNCE_MS 180

#define RED_MIN_RAW 500
#define GREEN_MIN_RAW 500
#define BLUE_MIN_RAW 500
#define RED_INTENSE_RAW 10000
#define DOMINANCE_MARGIN_RAW 200

// --- tempos configuráveis
#define COLOR_HOLD_MS 5000        // quanto tempo a matriz mantém a cor detectada (5s)
#define LED_TEST_HOLD_MS 5000     // quanto tempo o LED discreto fica em cada cor de teste (5s)
#define DETECTION_SETTLE_MS 150   // pequeno tempo antes de começar a coletar amostras
#define DETECTION_SAMPLES 4       // quantas amostras coletar para média
#define DETECTION_SAMPLE_INTERVAL_MS 25 // intervalo entre amostras

// =================== Globais ===================
ssd1306_t ssd;
ws2812_t ws;
static volatile uint32_t last_btn_a_ms = 0;

// -------------------- Utilidades de I2C (GY-33) --------------------
static inline void gy33_write_u8(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(I2C_SENS, GY33_ADDR, buf, 2, false);
}
static inline uint16_t gy33_read_u16(uint8_t reg)
{
    uint8_t b[2];
    i2c_write_blocking(I2C_SENS, GY33_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_SENS, GY33_ADDR, b, 2, false);
    return ((uint16_t)b[1] << 8) | b[0];
}
static void gy33_init(void)
{
    gy33_write_u8(REG_ENABLE, 0x03);
    gy33_write_u8(REG_ATIME, 0xD5);
    gy33_write_u8(REG_CONTROL, 0x00);
}
static void gy33_read_rgbc(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
    *c = gy33_read_u16(REG_CDATA);
    *r = gy33_read_u16(REG_RDATA);
    *g = gy33_read_u16(REG_GDATA);
    *b = gy33_read_u16(REG_BDATA);
}

// -------------------- OLED --------------------
static const char *nome_cor(uint16_t r, uint16_t g, uint16_t b)
{
    if (r >= RED_INTENSE_RAW && r > g + DOMINANCE_MARGIN_RAW && r > b + DOMINANCE_MARGIN_RAW)
        return "Vermelho (intenso)";
    if (r >= RED_MIN_RAW && r > g + DOMINANCE_MARGIN_RAW && r > b + DOMINANCE_MARGIN_RAW)
        return "Vermelho";
    if (g >= GREEN_MIN_RAW && g > r + DOMINANCE_MARGIN_RAW && g > b + DOMINANCE_MARGIN_RAW)
        return "Verde";
    if (b >= BLUE_MIN_RAW && b > r + DOMINANCE_MARGIN_RAW && b > g + DOMINANCE_MARGIN_RAW)
        return "Azul";
    if (r >= RED_MIN_RAW && g >= GREEN_MIN_RAW && b < BLUE_MIN_RAW)
        return "Amarelo";
    if (r >= RED_MIN_RAW && b >= BLUE_MIN_RAW && g < GREEN_MIN_RAW)
        return "Magenta";
    if (g >= GREEN_MIN_RAW && b >= BLUE_MIN_RAW && r < RED_MIN_RAW)
        return "Ciano";
    return "Mista";
}

static void oled_init_ui(void)
{
    i2c_init(I2C_DISP, 400 * 1000);
    gpio_set_function(I2C_DISP_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_DISP_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_DISP_SDA);
    gpio_pull_up(I2C_DISP_SCL);
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, OLED_ADDR, I2C_DISP);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}
static void oled_draw_status(uint16_t lux, uint16_t r, uint16_t g, uint16_t b, uint16_t c,
                             const char *cor_nome, const char *modo)
{
    char l1[32], l2[32], l3[32], l4[32], l5[32];
    snprintf(l1, sizeof(l1), "Modo: %s", modo);
    snprintf(l2, sizeof(l2), "Lux: %u", lux);
    snprintf(l3, sizeof(l3), "Cor: %s", cor_nome);
    snprintf(l4, sizeof(l4), "R:%u  G:%u", r, g);
    snprintf(l5, sizeof(l5), "B:%u  C:%u", b, c);
    ssd1306_fill(&ssd, 0);
    ssd1306_rect(&ssd, 0, 0, 127, 63, true, false);
    ssd1306_draw_string(&ssd, l1, 8, 6);
    ssd1306_draw_string(&ssd, l2, 8, 18);
    ssd1306_draw_string(&ssd, l3, 8, 30);
    ssd1306_draw_string(&ssd, l4, 8, 42);
    ssd1306_draw_string(&ssd, l5, 8, 54);
    ssd1306_send_data(&ssd);
}

// -------------------- WS2812 --------------------
static uint8_t clamp_u8(int v) { return (v < 0) ? 0 : (v > 255 ? 255 : (uint8_t)v); }
static uint8_t brightness_from_lux(uint16_t lux)
{
    float L = lux > 1000 ? 1000.0f : (float)lux;
    int b = (int)(5 + (L / 1000.0f) * 250.0f);
    return clamp_u8(b);
}
static void ws2812_fill_color(ws2812_t *w, uint8_t r, uint8_t g, uint8_t b)
{
    for (uint i = 0; i < WS2812_COUNT; i++)
        ws2812_set_pixel(w, i, r, g, b);
    ws2812_show(w);
}

// -------------------- Buzzer (PWM) --------------------
static uint slice_buzzer;
static void buzzer_init(void)
{
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    slice_buzzer = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_enabled(slice_buzzer, true);
    pwm_set_wrap(slice_buzzer, 2000);
}
static void buzzer_tone(uint32_t freq_hz, uint16_t ms)
{
    if (freq_hz == 0)
    {
        pwm_set_gpio_level(BUZZER_PIN, 0);
        sleep_ms(ms);
        return;
    }
    uint32_t f_sys = clock_get_hz(clk_sys);
    float div = (float)f_sys / (freq_hz * (2001));
    if (div < 1.0f) div = 1.0f;
    if (div > 255.0f) div = 255.0f;
    pwm_set_clkdiv(slice_buzzer, div);
    pwm_set_gpio_level(BUZZER_PIN, 1000);
    sleep_ms(ms);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

// -------------------- LED RGB Discreto --------------------
static inline void led_rgb_init(void)
{
    gpio_init(LED_R_PIN);
    gpio_init(LED_G_PIN);
    gpio_init(LED_B_PIN);
    gpio_set_dir(LED_R_PIN, GPIO_OUT);
    gpio_set_dir(LED_G_PIN, GPIO_OUT);
    gpio_set_dir(LED_B_PIN, GPIO_OUT);
}
static inline void led_rgb_set_raw(bool r_on, bool g_on, bool b_on)
{
    bool r_lvl = LED_COMMON_ANODE ? !r_on : r_on;
    bool g_lvl = LED_COMMON_ANODE ? !g_on : g_on;
    bool b_lvl = LED_COMMON_ANODE ? !b_on : b_on;
    gpio_put(LED_R_PIN, r_lvl);
    gpio_put(LED_G_PIN, g_lvl);
    gpio_put(LED_B_PIN, b_lvl);
}
static inline void led_rgb_off(void) { led_rgb_set_raw(false, false, false); }
static inline void led_rgb_red(void) { led_rgb_set_raw(true, false, false); }
static inline void led_rgb_grn(void) { led_rgb_set_raw(false, true, false); }
static inline void led_rgb_blu(void) { led_rgb_set_raw(false, false, true); }
static inline void led_rgb_wht(void) { led_rgb_set_raw(true, true, true); }

// -------------------- Modo de Cor --------------------
typedef enum
{
    MODO_SENSOR = 0,
    MODO_VERMELHO,
    MODO_VERDE,
    MODO_AZUL,
    MODO_BRANCO,
    MODO_APAGADO,
    MODO__MAX
} modo_rgb_t;

static volatile modo_rgb_t modo_atual = MODO_SENSOR;
static const char *modo_nome(modo_rgb_t m)
{
    switch (m)
    {
    case MODO_SENSOR: return "Sensor";
    case MODO_VERMELHO: return "Vermelho";
    case MODO_VERDE: return "Verde";
    case MODO_AZUL: return "Azul";
    case MODO_BRANCO: return "Branco";
    case MODO_APAGADO: return "Apagado";
    default: return "?";
    }
}

static void btn_a_irq(uint gpio, uint32_t events)
{
    if (gpio != BTN_A_PIN) return;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_btn_a_ms < BTN_DEBOUNCE_MS) return;
    last_btn_a_ms = now;
    modo_atual = (modo_rgb_t)((modo_atual + 1) % MODO__MAX);
}
static void bootsel_irq(uint gpio, uint32_t events)
{
    if (gpio == BTN_B_PIN) reset_usb_boot(0, 0);
}

// -------------------- Detecção: enum e state-machine --------------------
typedef enum {
    DET_UNKNOWN = 0,
    DET_RED,
    DET_GREEN,
    DET_BLUE,
    DET_YELLOW,
    DET_MAGENTA,
    DET_CYAN,
    DET_WHITE
} detected_color_t;

typedef enum {
    DS_IDLE = 0,        // aguardando iniciar um teste
    DS_TEST_SETUP,      // acende LED discreto p/ cor de teste e registra timestamp
    DS_SETTLE,          // espera DETECTION_SETTLE_MS antes de começar amostras
    DS_SAMPLING,        // coletando amostras (DETECTION_SAMPLES) com intervalos
    DS_EVALUATE,        // avalia as amostras e decide cor detectada ou não
    DS_HOLD_MATRIX,     // mantém cor detectada na matriz por COLOR_HOLD_MS
    DS_DONE             // estado final temporário (mapear pra IDLE)
} detect_phase_t;

typedef struct {
    detect_phase_t phase;
    int test_index;             // 0: R, 1: G, 2: B, depois combos
    uint32_t phase_start_ms;    // quando entrou na fase
    uint32_t last_sample_ms;    // timestamp da última amostra
    int samples_collected;      // contador de amostras correntes
    uint32_t sr, sg, sb, sc;    // acumuladores de amostras
    detected_color_t last_applied; // última cor aplicada à matriz
    uint32_t hold_until_ms;     // timestamp até quando manter a matriz na cor
} detect_state_t;

// inicializa estado
static void detect_state_init(detect_state_t *s)
{
    s->phase = DS_IDLE;
    s->test_index = 0;
    s->phase_start_ms = 0;
    s->last_sample_ms = 0;
    s->samples_collected = 0;
    s->sr = s->sg = s->sb = s->sc = 0;
    s->last_applied = DET_UNKNOWN;
    s->hold_until_ms = 0;
}

// Helper: aplica cor detectada (usa lux para brilho) — mantém LED discreto e matriz sincronizados
static void apply_detected_color(uint16_t lux, detected_color_t d)
{
    uint8_t bright = brightness_from_lux(lux);
    switch (d)
    {
        case DET_RED:
            led_rgb_red();
            ws2812_fill_color(&ws, (uint8_t)((255 * bright) / 255), 0, 0);
            break;
        case DET_GREEN:
            led_rgb_grn();
            ws2812_fill_color(&ws, 0, (uint8_t)((255 * bright) / 255), 0);
            break;
        case DET_BLUE:
            led_rgb_blu();
            ws2812_fill_color(&ws, 0, 0, (uint8_t)((255 * bright) / 255));
            break;
        case DET_YELLOW:
            led_rgb_set_raw(true, true, false);
            ws2812_fill_color(&ws, (uint8_t)((255 * bright) / 255), (uint8_t)((255 * bright) / 255), 0);
            break;
        case DET_MAGENTA:
            led_rgb_set_raw(true, false, true);
            ws2812_fill_color(&ws, (uint8_t)((255 * bright) / 255), 0, (uint8_t)((255 * bright) / 255));
            break;
        case DET_CYAN:
            led_rgb_set_raw(false, true, true);
            ws2812_fill_color(&ws, 0, (uint8_t)((255 * bright) / 255), (uint8_t)((255 * bright) / 255));
            break;
        case DET_WHITE:
            led_rgb_wht();
            ws2812_fill_color(&ws, (uint8_t)((200 * bright) / 255), (uint8_t)((200 * bright) / 255), (uint8_t)((200 * bright) / 255));
            break;
        default:
            led_rgb_off();
            ws2812_fill_color(&ws, 40, 40, 40);
            break;
    }
}

// Decide cor a partir das médias acumuladas (mesma lógica de nome_cor)
static detected_color_t decide_color_from_avg(uint32_t sr, uint32_t sg, uint32_t sb)
{
    uint16_t r_avg = sr / (DETECTION_SAMPLES > 0 ? DETECTION_SAMPLES : 1);
    uint16_t g_avg = sg / (DETECTION_SAMPLES > 0 ? DETECTION_SAMPLES : 1);
    uint16_t b_avg = sb / (DETECTION_SAMPLES > 0 ? DETECTION_SAMPLES : 1);

    if (r_avg >= RED_INTENSE_RAW && r_avg > g_avg + DOMINANCE_MARGIN_RAW && r_avg > b_avg + DOMINANCE_MARGIN_RAW)
        return DET_RED;
    if (r_avg >= RED_MIN_RAW && r_avg > g_avg + DOMINANCE_MARGIN_RAW && r_avg > b_avg + DOMINANCE_MARGIN_RAW)
        return DET_RED;
    if (g_avg >= GREEN_MIN_RAW && g_avg > r_avg + DOMINANCE_MARGIN_RAW && g_avg > b_avg + DOMINANCE_MARGIN_RAW)
        return DET_GREEN;
    if (b_avg >= BLUE_MIN_RAW && b_avg > r_avg + DOMINANCE_MARGIN_RAW && b_avg > g_avg + DOMINANCE_MARGIN_RAW)
        return DET_BLUE;
    // combos (simples)
    if (r_avg >= RED_MIN_RAW && g_avg >= GREEN_MIN_RAW && b_avg < BLUE_MIN_RAW) return DET_YELLOW;
    if (r_avg >= RED_MIN_RAW && b_avg >= BLUE_MIN_RAW && g_avg < GREEN_MIN_RAW) return DET_MAGENTA;
    if (g_avg >= GREEN_MIN_RAW && b_avg >= BLUE_MIN_RAW && r_avg < RED_MIN_RAW) return DET_CYAN;
    if (r_avg >= RED_MIN_RAW && g_avg >= GREEN_MIN_RAW && b_avg >= BLUE_MIN_RAW) return DET_WHITE;
    return DET_UNKNOWN;
}

// Step não-bloqueante da máquina de detecção.
// Deve ser chamado frequentemente pelo loop principal.
// Retorna 1 se aplicou nova cor (e atualizou hold_until_ms), 0 caso contrário.
static int detect_state_step(detect_state_t *s, uint16_t lux)
{
    uint32_t now = to_ms_since_boot(get_absolute_time());

    switch (s->phase)
    {
        case DS_IDLE:
        {
            // inicia um novo ciclo de testes: R -> G -> B -> combos
            s->test_index = 0;
            s->phase = DS_TEST_SETUP;
            s->phase_start_ms = now;
            s->samples_collected = 0;
            s->sr = s->sg = s->sb = s->sc = 0;
            return 0;
        }

        case DS_TEST_SETUP:
        {
            // Garante matriz apagada para evitar feedback óptico
            ws2812_fill_color(&ws, 0, 0, 0);

            // Acende o LED discreto da cor de teste atual
            if (s->test_index == 0) led_rgb_red();
            else if (s->test_index == 1) led_rgb_grn();
            else if (s->test_index == 2) led_rgb_blu();
            else led_rgb_off();

            s->phase_start_ms = now;
            s->last_sample_ms = now;
            s->samples_collected = 0;
            s->sr = s->sg = s->sb = s->sc = 0;
            s->phase = DS_SETTLE;
            return 0;
        }

        case DS_SETTLE:
        {
            // espera um pequeno tempo para estabilização antes de amostras
            if (now - s->phase_start_ms >= DETECTION_SETTLE_MS)
            {
                s->phase = DS_SAMPLING;
                s->last_sample_ms = now; // pronto para primeira amostra
            }
            return 0;
        }

        case DS_SAMPLING:
        {
            // coleta uma amostra a cada DETECTION_SAMPLE_INTERVAL_MS até DETECTION_SAMPLES
            if ((int)(now - s->last_sample_ms) >= DETECTION_SAMPLE_INTERVAL_MS)
            {
                uint16_t rr, gg, bb, cc;
                gy33_read_rgbc(&rr, &gg, &bb, &cc);
                s->sr += rr;
                s->sg += gg;
                s->sb += bb;
                s->sc += cc;
                s->samples_collected++;
                s->last_sample_ms = now;
            }

            // se já coletou o suficiente, vai avaliar
            if (s->samples_collected >= DETECTION_SAMPLES)
            {
                s->phase = DS_EVALUATE;
            }
            return 0;
        }

        case DS_EVALUATE:
        {
            // Decide cor a partir das médias
            detected_color_t det = decide_color_from_avg(s->sr, s->sg, s->sb);

            if (det != DET_UNKNOWN)
            {
                // detectou uma cor dominantes nesta etapa (R/G/B ou combo)
                apply_detected_color(lux, det);
                s->last_applied = det;
                s->hold_until_ms = now + COLOR_HOLD_MS; // manter a matriz por COLOR_HOLD_MS
                s->phase = DS_HOLD_MATRIX;
                return 1; // aplicou nova cor
            }
            else
            {
                // se não detectou na cor simples, avançar para próximo teste
                s->test_index++;
                // se passou por R,G,B sem sucesso, testaremos combos sequencialmente
                if (s->test_index <= 2)
                {
                    s->phase = DS_TEST_SETUP; // próximo teste simples
                }
                else if (s->test_index == 3)
                {
                    // testar R+G (amarelo)
                    led_rgb_set_raw(true, true, false);
                    s->phase_start_ms = now;
                    s->last_sample_ms = now;
                    s->samples_collected = 0;
                    s->sr = s->sg = s->sb = s->sc = 0;
                    s->phase = DS_SETTLE;
                }
                else if (s->test_index == 4)
                {
                    // testar R+B (magenta)
                    led_rgb_set_raw(true, false, true);
                    s->phase_start_ms = now;
                    s->last_sample_ms = now;
                    s->samples_collected = 0;
                    s->sr = s->sg = s->sb = s->sc = 0;
                    s->phase = DS_SETTLE;
                }
                else if (s->test_index == 5)
                {
                    // testar G+B (ciano)
                    led_rgb_set_raw(false, true, true);
                    s->phase_start_ms = now;
                    s->last_sample_ms = now;
                    s->samples_collected = 0;
                    s->sr = s->sg = s->sb = s->sc = 0;
                    s->phase = DS_SETTLE;
                }
                else if (s->test_index == 6)
                {
                    // testar branco (todos altos)
                    led_rgb_set_raw(true, true, true);
                    s->phase_start_ms = now;
                    s->last_sample_ms = now;
                    s->samples_collected = 0;
                    s->sr = s->sg = s->sb = s->sc = 0;
                    s->phase = DS_SETTLE;
                }
                else
                {
                    // terminou todos os testes sem detectar nada -> volta para idle
                    led_rgb_off();
                    s->phase = DS_IDLE;
                }
            }
            return 0;
        }

        case DS_HOLD_MATRIX:
        {
            // mantém a cor aplicada na matriz até hold_until_ms expirar (não bloqueante)
            if ((int32_t)(to_ms_since_boot(get_absolute_time()) - s->hold_until_ms) >= 0)
            {
                // hold expirou, apagar LED discreto e reiniciar ciclo
                led_rgb_off();
                s->phase = DS_IDLE;
            }
            return 0;
        }

        default:
            s->phase = DS_IDLE;
            return 0;
    }
}

// -------------------- MAIN --------------------
int main(void)
{
    stdio_init_all();

    // Botões (BOOTSEL e modo)
    gpio_init(BTN_B_PIN);
    gpio_set_dir(BTN_B_PIN, GPIO_IN);
    gpio_pull_up(BTN_B_PIN);
    gpio_set_irq_enabled_with_callback(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true, &bootsel_irq);

    gpio_init(BTN_A_PIN);
    gpio_set_dir(BTN_A_PIN, GPIO_IN);
    gpio_pull_up(BTN_A_PIN);
    gpio_set_irq_enabled_with_callback(BTN_A_PIN, GPIO_IRQ_EDGE_FALL, true, &btn_a_irq);

    // OLED
    oled_init_ui();

    // I2C sensores
    i2c_init(I2C_SENS, 400 * 1000);
    gpio_set_function(I2C_SENS_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SENS_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SENS_SDA);
    gpio_pull_up(I2C_SENS_SCL);

    // inicializa sensores e periféricos
    bh1750_power_on(I2C_SENS);
    gy33_init();
    ws2812_init(&ws, WS2812_PIN, WS2812_COUNT);
    ws2812_clear(&ws);
    buzzer_init();
    led_rgb_init();
    led_rgb_off();
    sleep_ms(100); // pequena pausa inicial

    // init estado de detecção
    detect_state_t detector;
    detect_state_init(&detector);

    // histórico para evitar re-aplicar cor idêntica
    detected_color_t last_detected_applied = DET_UNKNOWN;

    while (true)
    {
        // leitura ambiente + leitura instantânea do GY-33 para exibição/debug
        uint16_t lux = bh1750_read_measurement(I2C_SENS);
        uint16_t r, g, b, c;
        gy33_read_rgbc(&r, &g, &b, &c);
        const char *ncor = nome_cor(r, g, b);

        printf("Lux=%u | R=%u G=%u B=%u C=%u | Modo=%s\n", lux, r, g, b, c, modo_nome(modo_atual));

        if (modo_atual == MODO_SENSOR)
        {
            // passo a máquina de estados não bloqueante;
            // se detect_state_step aplicar nova cor, emite buzzer se for vermelho intenso.
            int applied = detect_state_step(&detector, lux);
            if (applied)
            {
                // detector.last_applied contém a cor aplicada
                last_detected_applied = detector.last_applied;
                if (last_detected_applied == DET_RED)
                    buzzer_tone(1200, BUZZER_ALERT_MS);
            }
        }
        else
        {
            // Modos manuais comportam-se como antes (imediatos)
            switch (modo_atual)
            {
                case MODO_VERMELHO:
                    led_rgb_red();
                    ws2812_fill_color(&ws, 255, 0, 0);
                    last_detected_applied = DET_RED;
                    break;
                case MODO_VERDE:
                    led_rgb_grn();
                    ws2812_fill_color(&ws, 0, 255, 0);
                    last_detected_applied = DET_GREEN;
                    break;
                case MODO_AZUL:
                    led_rgb_blu();
                    ws2812_fill_color(&ws, 0, 0, 255);
                    last_detected_applied = DET_BLUE;
                    break;
                case MODO_BRANCO:
                    led_rgb_wht();
                    ws2812_fill_color(&ws, 255, 255, 255);
                    last_detected_applied = DET_WHITE;
                    break;
                case MODO_APAGADO:
                    led_rgb_off();
                    ws2812_fill_color(&ws, 0, 0, 0);
                    last_detected_applied = DET_UNKNOWN;
                    break;
                default:
                    break;
            }
        }

        // Alertas gerais (não bloqueantes)
        if (lux < LUX_LOW_THRESHOLD)
        {
            buzzer_tone(2200, BUZZER_ALERT_MS);
        }

        if (r >= RED_INTENSE_RAW && r > g + DOMINANCE_MARGIN_RAW && r > b + DOMINANCE_MARGIN_RAW)
        {
            buzzer_tone(1200, BUZZER_ALERT_MS);
        }

        // Atualiza OLED (rápido)
        oled_draw_status(lux, r, g, b, c, ncor, modo_nome(modo_atual));

        // Pequena pausa para reduzir uso de CPU (não bloqueante pesado)
        sleep_ms(LOOP_DELAY_MS);
    }

    return 0;
}
