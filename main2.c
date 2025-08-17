#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/bootrom.h"

// ==== OLED ====
#include "ssd1306.h"
#include "font.h"

// ==== BH1750 (lux) ====
#include "bh1750_light_sensor.h"  // usa funções conforme seus arquivos :contentReference[oaicite:4]{index=4}

// ==== WS2812 PIO ====
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"   // gere com pioasm; normalmente já existe no seu projeto (BitDogLab)

// ==== GY-33 (TCS34725 – RGB/Clear) ====
#define GY33_I2C_ADDR 0x29
// registradores do TCS34725 (iguais aos usados no seu exemplo) :contentReference[oaicite:5]{index=5}
#define ENABLE_REG 0x80
#define ATIME_REG  0x81
#define CONTROL_REG 0x8F
#define CDATA_REG  0x94
#define RDATA_REG  0x96
#define GDATA_REG  0x98
#define BDATA_REG  0x9A

// =================== MAPEAMENTO DE PINOS (BitDogLab) ===================
// I2C0 para SENSORES (BH1750 e GY-33)
#define I2C_SENS       i2c0
#define I2C_SENS_SDA   0
#define I2C_SENS_SCL   1

// I2C1 para DISPLAY (SSD1306)
#define I2C_DISP       i2c1
#define I2C_DISP_SDA   14
#define I2C_DISP_SCL   15
#define OLED_ADDR      0x3C

// Matriz WS2812 5x5 (DIN padrão da BitDogLab costuma ser GPIO 7)
#define WS2812_PIN     7
#define WS2812_NUM_LEDS 25
#define WS2812_IS_RGBW  false

// Buzzer (PWM) – ajuste se necessário (comum usar GPIO 21 na BitDogLab)
#define BUZZER_PIN     21

// Botão A (já usado nos seus exemplos)
#define BTN_A_PIN      5

// Botão B -> BOOTSEL
#define BTN_B_PIN      6

// =================== PARÂMETROS DE LÓGICA ===================
#define LUX_LOW_THRESHOLD      50     // alerta luminosidade baixa
#define RED_STRONG_THRESHOLD   1500   // “vermelho intenso” (ajuste pela sua cena de luz)
#define BUZZER_ALERT_MS        150
#define LOOP_DELAY_MS          300

// =================== GLOBAIS ===================
ssd1306_t ssd;
PIO ws2812_pio;
uint sm_ws;

// =================== UTIL: BOOTSEL via botão B ===================
void gpio_irq_handler(uint gpio, uint32_t events) {
    if (gpio == BTN_B_PIN) reset_usb_boot(0, 0);
}

// =================== BH1750 ===================
// Suas funções já vêm de bh1750_light_sensor.(h/c) (power on + read) :contentReference[oaicite:6]{index=6}

// =================== GY-33 (TCS34725) via I2C0 ===================
static inline void gy33_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    i2c_write_blocking(I2C_SENS, GY33_I2C_ADDR, buf, 2, false);
}

static inline uint16_t gy33_read16(uint8_t reg) {
    uint8_t b[2];
    i2c_write_blocking(I2C_SENS, GY33_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_SENS, GY33_I2C_ADDR, b, 2, false);
    return ((uint16_t)b[1] << 8) | b[0];
}

static void gy33_init(void) {
    // Mesma sequência do seu exemplo (ENABLE/ATIME/CONTROL). ATIME 0xD5 ~ 103.2ms
    gy33_write_reg(ENABLE_REG,  0x03);
    gy33_write_reg(ATIME_REG,   0xD5);  // integração ~103 ms (do seu material) :contentReference[oaicite:7]{index=7}
    gy33_write_reg(CONTROL_REG, 0x00);  // ganho 1x
}

static void gy33_read_rgbc(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c) {
    *c = gy33_read16(CDATA_REG);
    *r = gy33_read16(RDATA_REG);
    *g = gy33_read16(GDATA_REG);
    *b = gy33_read16(BDATA_REG);
}

// =================== OLED ===================
static void oled_init_ui(void) {
    // I2C1 @ 400kHz
    i2c_init(I2C_DISP, 400 * 1000);
    gpio_set_function(I2C_DISP_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_DISP_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_DISP_SDA);
    gpio_pull_up(I2C_DISP_SCL);
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, OLED_ADDR, I2C_DISP);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}

static void oled_draw_status(uint16_t lux, uint16_t r, uint16_t g, uint16_t b, uint16_t c, const char* cor_nome) {
    char line1[32], line2[32], line3[32];
    snprintf(line1, sizeof(line1), "Lux: %u", lux);
    snprintf(line2, sizeof(line2), "R:%u G:%u", r, g);
    snprintf(line3, sizeof(line3), "B:%u C:%u", b, c);

    bool on = true;
    ssd1306_fill(&ssd, !on);
    ssd1306_rect(&ssd, 2, 2, 124, 60, on, !on);
    ssd1306_draw_string(&ssd, "EMBARCATECH", 14, 6);
    ssd1306_draw_string(&ssd, line1, 8, 20);
    ssd1306_draw_string(&ssd, line2, 8, 32);
    ssd1306_draw_string(&ssd, line3, 8, 44);
    // cor detectada (nome)
    ssd1306_draw_string(&ssd, cor_nome, 82, 20);
    ssd1306_send_data(&ssd);
}

// =================== BUZZER (PWM) ===================
static uint slice_buzzer;

static void buzzer_init(void) {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    slice_buzzer = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_enabled(slice_buzzer, true);
}

static void buzzer_tone_hz(uint32_t freq, uint16_t ms) {
    if (freq == 0) { pwm_set_enabled(slice_buzzer, false); sleep_ms(ms); pwm_set_enabled(slice_buzzer, true); return; }
    uint32_t f_sys = clock_get_hz(clk_sys);
    uint32_t div16 = f_sys / (freq * 2000);   // alvo: ~50% duty com wrap 2000
    if (div16 < 16) div16 = 16;
    pwm_set_clkdiv_int_frac(slice_buzzer, div16/16, div16 & 0xF);
    pwm_set_wrap(slice_buzzer, 2000);
    pwm_set_gpio_level(BUZZER_PIN, 1000);
    sleep_ms(ms);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

// =================== WS2812 ===================
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    // Formato GRB para WS2812
    return
        ((uint32_t)(g) << 16) |
        ((uint32_t)(r) << 8)  |
        ((uint32_t)(b) << 0);
}

static uint8_t clamp_u8(int v) { return (v < 0) ? 0 : (v > 255 ? 255 : (uint8_t)v); }

static void ws2812_init(void) {
    ws2812_pio = pio0;
    uint offset = pio_add_program(ws2812_pio, &ws2812_program);
    sm_ws = pio_claim_unused_sm(ws2812_pio, true);
    ws2812_program_init(ws2812_pio, sm_ws, offset, WS2812_PIN, 800000, WS2812_IS_RGBW);
}

// Brilho proporcional à luz: mapeia lux em [0..1000] -> brilho [5..255]
static uint8_t brightness_from_lux(uint16_t lux) {
    float l = (lux > 1000) ? 1000.0f : (float)lux;
    int b = (int)(5 + (l/1000.0f) * 250.0f);
    return clamp_u8(b);
}

// Converte RGB (16 bits do GY-33) para 8 bits + aplica brilho
static void ws2812_show_color(uint16_t r16, uint16_t g16, uint16_t b16, uint8_t brightness) {
    // normaliza 16b -> 8b (shift 8)
    uint8_t r = (uint8_t)(r16 >> 8);
    uint8_t g = (uint8_t)(g16 >> 8);
    uint8_t b = (uint8_t)(b16 >> 8);

    // aplica brilho (escala simples)
    r = (uint8_t)((r * brightness) / 255);
    g = (uint8_t)((g * brightness) / 255);
    b = (uint8_t)((b * brightness) / 255);

    uint32_t pix = urgb_u32(r, g, b);
    for (int i = 0; i < WS2812_NUM_LEDS; i++) {
        pio_sm_put_blocking(ws2812_pio, sm_ws, pix << 8u);
    }
}

// Heurística simples de nome da cor para exibir no OLED
static const char* nome_cor(uint16_t r, uint16_t g, uint16_t b) {
    // normaliza
    float rf = r / 65535.0f, gf = g / 65535.0f, bf = b / 65535.0f;
    if (rf > 0.5f && gf < 0.3f && bf < 0.3f) return "Vermelho";
    if (gf > 0.5f && rf < 0.3f && bf < 0.3f) return "Verde";
    if (bf > 0.5f && rf < 0.3f && gf < 0.3f) return "Azul";
    if (rf > 0.5f && gf > 0.5f && bf < 0.3f) return "Amarelo";
    if (rf > 0.5f && bf > 0.5f && gf < 0.3f) return "Magenta";
    if (gf > 0.5f && bf > 0.5f && rf < 0.3f) return "Ciano";
    return "Mista";
}

int main() {
    stdio_init_all();

    // BOOTSEL via Botão B
    gpio_init(BTN_B_PIN);
    gpio_set_dir(BTN_B_PIN, GPIO_IN);
    gpio_pull_up(BTN_B_PIN);
    gpio_set_irq_enabled_with_callback(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Botão A (pode usar depois para alternar modos, se quiser)
    gpio_init(BTN_A_PIN);
    gpio_set_dir(BTN_A_PIN, GPIO_IN);
    gpio_pull_up(BTN_A_PIN);

    // ==== OLED ====
    oled_init_ui();

    // ==== I2C0 sensores ====
    i2c_init(I2C_SENS, 400 * 1000);
    gpio_set_function(I2C_SENS_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SENS_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SENS_SDA);
    gpio_pull_up(I2C_SENS_SCL);

    // ==== BH1750 ====
    bh1750_power_on(I2C_SENS); // conforme seu driver :contentReference[oaicite:8]{index=8}

    // ==== GY-33 ====
    gy33_init();               // mesma ideia do seu exemplo :contentReference[oaicite:9]{index=9}

    // ==== WS2812 ====
    ws2812_init();

    // ==== BUZZER ====
    buzzer_init();

    sleep_ms(500);

    while (true) {
        // 1) Lê sensores
        uint16_t lux = bh1750_read_measurement(I2C_SENS); // envia comando H-RES e lê (div/1.2) :contentReference[oaicite:10]{index=10}
        uint16_t r, g, b, c;
        gy33_read_rgbc(&r, &g, &b, &c);                   // leitura RGB/Clear via I2C0 :contentReference[oaicite:11]{index=11}

        printf("Lux=%u | R=%u G=%u B=%u C=%u\n", lux, r, g, b, c);

        // 2) Atualiza matriz WS2812 com cor detectada e brilho proporcional à luz
        uint8_t bright = brightness_from_lux(lux);
        ws2812_show_color(r, g, b, bright);

        // 3) Alertas do buzzer:
        //    - luminosidade baixa
        if (lux < LUX_LOW_THRESHOLD) {
            buzzer_tone_hz(2000, BUZZER_ALERT_MS);
        }
        //    - vermelho intenso (r muito acima de g e b)
        if (r > RED_STRONG_THRESHOLD && r > (g + 100) && r > (b + 100)) {
            buzzer_tone_hz(1200, BUZZER_ALERT_MS);
        }

        // 4) OLED com dados
        const char* ncor = nome_cor(r, g, b);
        oled_draw_status(lux, r, g, b, c, ncor);

        sleep_ms(LOOP_DELAY_MS);
    }
    return 0;
}