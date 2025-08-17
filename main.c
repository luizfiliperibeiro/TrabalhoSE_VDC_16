// main.c — Projeto Orientado (GY-33 + BH1750 + OLED + WS2812 + Buzzer PWM + LED RGB via Botão A)
// BitDogLab + RP2040 (Pico W)

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
// Sensores no I2C0 (via repetidor/booster) — sem mux
#define I2C_SENS         i2c0
#define I2C_SENS_SDA     0
#define I2C_SENS_SCL     1

// Display no I2C1
#define I2C_DISP         i2c1
#define I2C_DISP_SDA     14
#define I2C_DISP_SCL     15
#define OLED_ADDR        0x3C

// =================== GY-33 / TCS34725 (I2C0) ===================
#define GY33_ADDR        0x29
#define REG_ENABLE       0x80
#define REG_ATIME        0x81
#define REG_CONTROL      0x8F
#define REG_CDATA        0x94
#define REG_RDATA        0x96
#define REG_GDATA        0x98
#define REG_BDATA        0x9A

// =================== WS2812 ===================
#define WS2812_PIN       7
#define WS2812_COUNT     25

// =================== Buzzer PWM ===================
#define BUZZER_PIN       21

// =================== Botões ===================
#define BTN_A_PIN        5   // alterna modos LED RGB
#define BTN_B_PIN        6   // BOOTSEL

// =================== LED RGB Discreto ===================
// AJUSTE A PINAGEM CONFORME SUA BITDOGLAB/EXEMPLO DO PROF.
// Aqui usamos: R=GP11, G=GP12, B=GP13
#define LED_R_PIN        13
#define LED_G_PIN        11
#define LED_B_PIN        12
// Defina para 1 se seu LED RGB for "common-anode" (nível ativo invertido)
#define LED_COMMON_ANODE 0

// =================== Parâmetros de Lógica ===================
#define LUX_LOW_THRESHOLD      10
#define LOOP_DELAY_MS          300
#define BUZZER_ALERT_MS        150
#define BTN_DEBOUNCE_MS        180

// --- Limiares de detecção de cor (brutos 16-bit, 0..65535) ---
#define RED_MIN_RAW            500
#define GREEN_MIN_RAW          500
#define BLUE_MIN_RAW           500
#define RED_INTENSE_RAW        10000
#define DOMINANCE_MARGIN_RAW   200   // margem para evitar empate/oscilação

// =================== Globais ===================
ssd1306_t ssd;
ws2812_t  ws;
static volatile uint32_t last_btn_a_ms = 0;

// -------------------- Utilidades de I2C (GY-33) --------------------
static inline void gy33_write_u8(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    i2c_write_blocking(I2C_SENS, GY33_ADDR, buf, 2, false);
}
static inline uint16_t gy33_read_u16(uint8_t reg) {
    uint8_t b[2];
    i2c_write_blocking(I2C_SENS, GY33_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_SENS, GY33_ADDR, b, 2, false);
    return ((uint16_t)b[1] << 8) | b[0];
}
static void gy33_init(void) {
    gy33_write_u8(REG_ENABLE,  0x03); // Power ON + ADC enable
    gy33_write_u8(REG_ATIME,   0xD5); // ~103.2 ms
    gy33_write_u8(REG_CONTROL, 0x00); // ganho 1x
}
static void gy33_read_rgbc(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c) {
    *c = gy33_read_u16(REG_CDATA);
    *r = gy33_read_u16(REG_RDATA);
    *g = gy33_read_u16(REG_GDATA);
    *b = gy33_read_u16(REG_BDATA);
}

// -------------------- OLED --------------------
static const char* nome_cor(uint16_t r, uint16_t g, uint16_t b) {
    // Regras prioritárias com limiares absolutos e margem de dominância
    if (r >= RED_INTENSE_RAW && r > g + DOMINANCE_MARGIN_RAW && r > b + DOMINANCE_MARGIN_RAW)
        return "Vermelho (intenso)";

    if (r >= RED_MIN_RAW   && r > g + DOMINANCE_MARGIN_RAW && r > b + DOMINANCE_MARGIN_RAW) return "Vermelho";
    if (g >= GREEN_MIN_RAW && g > r + DOMINANCE_MARGIN_RAW && g > b + DOMINANCE_MARGIN_RAW) return "Verde";
    if (b >= BLUE_MIN_RAW  && b > r + DOMINANCE_MARGIN_RAW && b > g + DOMINANCE_MARGIN_RAW) return "Azul";

    // Combinações aproximadas (usam mesmos limiares mínimos)
    if (r >= RED_MIN_RAW && g >= GREEN_MIN_RAW && b <  BLUE_MIN_RAW)  return "Amarelo";
    if (r >= RED_MIN_RAW && b >= BLUE_MIN_RAW  && g <  GREEN_MIN_RAW) return "Magenta";
    if (g >= GREEN_MIN_RAW && b >= BLUE_MIN_RAW && r < RED_MIN_RAW)   return "Ciano";

    return "Mista";
}

static void oled_init_ui(void) {
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
static void oled_draw_status(uint16_t lux, uint16_t r, uint16_t g, uint16_t b, uint16_t c, const char* cor_nome, const char* modo) {
    char l1[32], l2[32], l3[32], l4[24];
    snprintf(l1, sizeof(l1), "Lux: %u", lux);
    snprintf(l2, sizeof(l2), "R:%u G:%u", r, g);
    snprintf(l3, sizeof(l3), "B:%u C:%u", b, c);
    snprintf(l4, sizeof(l4), "Modo: %s", modo);

    bool on = true;
    ssd1306_fill(&ssd, !on);
    ssd1306_rect(&ssd, 2, 2, 124, 60, on, !on);
    ssd1306_draw_string(&ssd, "EMBARCATECH", 14, 6);
    ssd1306_draw_string(&ssd, l1, 8, 18);
    ssd1306_draw_string(&ssd, l2, 8, 30);
    ssd1306_draw_string(&ssd, l3, 8, 42);
    ssd1306_draw_string(&ssd, cor_nome, 82, 18);
    ssd1306_draw_string(&ssd, l4, 8, 54);
    ssd1306_send_data(&ssd);
}

// Mapeia o nome de cor (classificação) para uma cor discreta RGB (8-bit base)
// Retorna true se mapeou; false para "desconhecido" (útil para apagar)
static bool map_color_name_to_rgb(const char* nome, uint8_t* r, uint8_t* g, uint8_t* b) {
    // Base sem brilho (vamos aplicar brilho depois, se quiser manter)
    if (!nome) return false;

    if (!strcmp(nome, "Vermelho (intenso)") || !strcmp(nome, "Vermelho")) {
        *r = 255; *g = 0;   *b = 0;   return true;
    }
    if (!strcmp(nome, "Verde")) {
        *r = 0;   *g = 255; *b = 0;   return true;
    }
    if (!strcmp(nome, "Azul")) {
        *r = 0;   *g = 0;   *b = 255; return true;
    }
    if (!strcmp(nome, "Amarelo")) {
        *r = 255; *g = 255; *b = 0;   return true;
    }
    if (!strcmp(nome, "Magenta")) {
        *r = 255; *g = 0;   *b = 255; return true;
    }
    if (!strcmp(nome, "Ciano")) {
        *r = 0;   *g = 255; *b = 255; return true;
    }
    if (!strcmp(nome, "Mista")) {
        // pedido do usuário: "se for mista, acende branco"
        *r = 255; *g = 255; *b = 255; return true;
    }

    // Desconhecido/outros
    return false;
}

// -------------------- WS2812 --------------------
static uint8_t clamp_u8(int v) { return (v < 0) ? 0 : (v > 255 ? 255 : (uint8_t)v); }
// brilho ~ mapeia 0..1000 lux para 5..255
static uint8_t brightness_from_lux(uint16_t lux) {
    float L = lux > 1000 ? 1000.0f : (float)lux;
    int b = (int)(5 + (L/1000.0f)*250.0f);
    return clamp_u8(b);
}
static void ws2812_fill_color(ws2812_t* w, uint8_t r, uint8_t g, uint8_t b) {
    for (uint i = 0; i < WS2812_COUNT; i++) ws2812_set_pixel(w, i, r, g, b);
    ws2812_show(w);
}

// -------------------- Buzzer (PWM) --------------------
static uint slice_buzzer;
static void buzzer_init(void) {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    slice_buzzer = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_enabled(slice_buzzer, true);
    pwm_set_wrap(slice_buzzer, 2000);
}
static void buzzer_tone(uint32_t freq_hz, uint16_t ms) {
    if (freq_hz == 0) { pwm_set_gpio_level(BUZZER_PIN, 0); sleep_ms(ms); return; }
    uint32_t f_sys = clock_get_hz(clk_sys);
    uint32_t div16 = f_sys / (freq_hz * (2000 + 1));
    if (div16 < 16) div16 = 16;
    pwm_set_clkdiv_int_frac(slice_buzzer, div16/16, div16 & 0xF);
    pwm_set_gpio_level(BUZZER_PIN, 1000); // ~50%
    sleep_ms(ms);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

// -------------------- LED RGB Discreto --------------------
static inline void led_rgb_init(void) {
    gpio_init(LED_R_PIN); gpio_init(LED_G_PIN); gpio_init(LED_B_PIN);
    gpio_set_dir(LED_R_PIN, GPIO_OUT);
    gpio_set_dir(LED_G_PIN, GPIO_OUT);
    gpio_set_dir(LED_B_PIN, GPIO_OUT);
}
static inline void led_rgb_set_raw(bool r_on, bool g_on, bool b_on) {
    bool r_lvl = LED_COMMON_ANODE ? !r_on : r_on;
    bool g_lvl = LED_COMMON_ANODE ? !g_on : g_on;
    bool b_lvl = LED_COMMON_ANODE ? !b_on : b_on;
    gpio_put(LED_R_PIN, r_lvl);
    gpio_put(LED_G_PIN, g_lvl);
    gpio_put(LED_B_PIN, b_lvl);
}
static inline void led_rgb_off(void) { led_rgb_set_raw(false,false,false); }
static inline void led_rgb_red(void) { led_rgb_set_raw(true,false,false); }
static inline void led_rgb_grn(void) { led_rgb_set_raw(false,true,false); }
static inline void led_rgb_blu(void) { led_rgb_set_raw(false,false,true); }
static inline void led_rgb_wht(void) { led_rgb_set_raw(true,true,true); }

// -------------------- Modo de Cor (controlado pelo Botão A) --------------------
typedef enum {
    MODO_SENSOR = 0, // usa cor do GY-33 + brilho do BH1750
    MODO_VERMELHO,
    MODO_VERDE,
    MODO_AZUL,
    MODO_BRANCO,
    MODO_APAGADO,
    MODO__MAX
} modo_rgb_t;

static volatile modo_rgb_t modo_atual = MODO_SENSOR;
static const char* modo_nome(modo_rgb_t m) {
    switch(m) {
        case MODO_SENSOR:   return "Sensor";
        case MODO_VERMELHO: return "Vermelho";
        case MODO_VERDE:    return "Verde";
        case MODO_AZUL:     return "Azul";
        case MODO_BRANCO:   return "Branco";
        case MODO_APAGADO:  return "Apagado";
        default:            return "?";
    }
}

// IRQ do Botão A com debounce simples
static void btn_a_irq(uint gpio, uint32_t events) {
    if (gpio != BTN_A_PIN) return;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_btn_a_ms < BTN_DEBOUNCE_MS) return;
    last_btn_a_ms = now;

    // Próximo modo
    modo_atual = (modo_rgb_t)((modo_atual + 1) % MODO__MAX);
}

// -------------------- BOOTSEL via Botão B --------------------
static void bootsel_irq(uint gpio, uint32_t events) {
    if (gpio == BTN_B_PIN) reset_usb_boot(0, 0);
}

// -------------------- MAIN --------------------
int main(void) {
    stdio_init_all();

    // BOOTSEL (BTN_B)
    gpio_init(BTN_B_PIN);
    gpio_set_dir(BTN_B_PIN, GPIO_IN);
    gpio_pull_up(BTN_B_PIN);
    gpio_set_irq_enabled_with_callback(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true, &bootsel_irq);

    // Botão A (modo RGB)
    gpio_init(BTN_A_PIN);
    gpio_set_dir(BTN_A_PIN, GPIO_IN);
    gpio_pull_up(BTN_A_PIN);
    gpio_set_irq_enabled_with_callback(BTN_A_PIN, GPIO_IRQ_EDGE_FALL, true, &btn_a_irq);

    // OLED (I2C1)
    oled_init_ui();

    // I2C0 sensores
    i2c_init(I2C_SENS, 400 * 1000);
    gpio_set_function(I2C_SENS_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SENS_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SENS_SDA);
    gpio_pull_up(I2C_SENS_SCL);

    // BH1750
    bh1750_power_on(I2C_SENS);

    // GY-33 (TCS34725)
    gy33_init();

    // WS2812
    ws2812_init(&ws, WS2812_PIN, WS2812_COUNT);
    ws2812_clear(&ws);

    // Buzzer
    buzzer_init();

    // LED RGB discreto
    led_rgb_init();
    led_rgb_off();

    sleep_ms(300);

    while (true) {
        // 1) Sensores
        uint16_t lux = bh1750_read_measurement(I2C_SENS);
        uint16_t r, g, b, c;
        gy33_read_rgbc(&r, &g, &b, &c);
        printf("Lux=%u | R=%u G=%u B=%u C=%u | Modo=%s\n", lux, r, g, b, c, modo_nome(modo_atual));

        // 2) Seleção de cor conforme modo
        uint8_t r8, g8_, b8;
        const char* ncor = nome_cor(r, g, b);

                if (modo_atual == MODO_SENSOR) {
            // Classificação pelo nome_cor(...)
            const char* ncor = nome_cor(r, g, b);

            // Brilho por lux 
            uint8_t bright = brightness_from_lux(lux);

            uint8_t baseR, baseG, baseB;
            if (map_color_name_to_rgb(ncor, &baseR, &baseG, &baseB)) {
                // Aplica brilho na cor discreta
                uint8_t r8  = (uint8_t)((baseR * bright) / 255);
                uint8_t g8_ = (uint8_t)((baseG * bright) / 255);
                uint8_t b8  = (uint8_t)((baseB * bright) / 255);
                ws2812_fill_color(&ws, r8, g8_, b8);
            } else {
                // Sem cor detectada → apaga a matriz
                ws2812_fill_color(&ws, 0, 0, 0);
            }

            // LED RGB discreto: mantém a mesma lógica de limiar/dominância
            if (r >= RED_INTENSE_RAW && r > g + DOMINANCE_MARGIN_RAW && r > b + DOMINANCE_MARGIN_RAW)      led_rgb_red();
            else if (r >= RED_MIN_RAW   && r > g + DOMINANCE_MARGIN_RAW && r > b + DOMINANCE_MARGIN_RAW)    led_rgb_red();
            else if (g >= GREEN_MIN_RAW && g > r + DOMINANCE_MARGIN_RAW && g > b + DOMINANCE_MARGIN_RAW)    led_rgb_grn();
            else if (b >= BLUE_MIN_RAW  && b > r + DOMINANCE_MARGIN_RAW && b > g + DOMINANCE_MARGIN_RAW)    led_rgb_blu();
            else                                                                                              led_rgb_wht(); // opcional: pode trocar por led_rgb_off()

        } else {
            switch (modo_atual) {
                case MODO_VERMELHO: led_rgb_red();  ws2812_fill_color(&ws, 255,   0,   0); break;
                case MODO_VERDE:    led_rgb_grn();  ws2812_fill_color(&ws,   0, 255,   0); break;
                case MODO_AZUL:     led_rgb_blu();  ws2812_fill_color(&ws,   0,   0, 255); break;
                case MODO_BRANCO:   led_rgb_wht();  ws2812_fill_color(&ws, 255, 255, 255); break;
                case MODO_APAGADO:  led_rgb_off();  ws2812_fill_color(&ws,   0,   0,   0); break;
                default: break;
            }
        }

        // 3) Buzzer — alertas por lux baixo e vermelho intenso
        if (lux < LUX_LOW_THRESHOLD) {
            buzzer_tone(2200, BUZZER_ALERT_MS); // lux baixo
        }
        if (r >= RED_INTENSE_RAW && r > g + DOMINANCE_MARGIN_RAW && r > b + DOMINANCE_MARGIN_RAW) {
            buzzer_tone(1200, BUZZER_ALERT_MS); // vermelho intenso
        }

        // 4) OLED
        oled_draw_status(lux, r, g, b, c, ncor, modo_nome(modo_atual));

        sleep_ms(LOOP_DELAY_MS);
    }
    return 0;
}
// Fim do código principal