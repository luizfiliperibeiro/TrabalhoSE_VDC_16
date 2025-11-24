// main.c — Depuração do sensor de cor GY-33 (TCS34725) com PicoProbe
// BitDogLab + RP2040 (Pico W)
// - Usa apenas o sensor de cor (GY-33) via I2C0 (GPIO 0 e 1)
// - Saída de debug pela UART0 nos GPIO 16 (TX) e 17 (RX)
//   -> CMakeLists: pico_enable_stdio_usb(medidor_luz_cor 0)
//                  pico_enable_stdio_uart(medidor_luz_cor 1)

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

// -------------------- Configurações de hardware --------------------

// UART de debug (PicoProbe -> BitDogLab, conforme slides do professor)
#define UART_TX_PIN   16
#define UART_RX_PIN   17
#define BAUD_RATE     115200

// LED para indicação visual (LED discreto da BitDogLab)
#define LED_PIN       13
#define LED_ON        1
#define LED_OFF       0

// I2C para o GY-33 (I2C0 nos pinos 0 e 1 da BitDogLab)
#define I2C_SENS      i2c0
#define I2C_SENS_SDA  0
#define I2C_SENS_SCL  1

// Endereço do GY-33 (TCS34725)
#define GY33_ADDR     0x29

// Registradores principais do TCS34725 (já com o bit de comando 0x80)
#define REG_ENABLE    0x80
#define REG_ATIME     0x81
#define REG_CONTROL   0x8F
#define REG_CDATA     0x94
#define REG_RDATA     0x96
#define REG_GDATA     0x98
#define REG_BDATA     0x9A

// Limiares simples para classificação da cor (valores brutos do sensor)
#define RED_MIN_RAW          500
#define GREEN_MIN_RAW        500
#define BLUE_MIN_RAW         500
#define RED_INTENSE_RAW      10000
#define DOMINANCE_MARGIN_RAW 200

// Período entre leituras (ms)
#define LOOP_DELAY_MS  200

// -------------------- GY-33 / TCS34725: funções auxiliares --------------------

static inline void gy33_write_u8(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(I2C_SENS, GY33_ADDR, buf, 2, false);
}

static inline uint16_t gy33_read_u16(uint8_t reg) {
    uint8_t b[2];
    // Primeiro manda o endereço do registrador
    i2c_write_blocking(I2C_SENS, GY33_ADDR, &reg, 1, true);
    // Depois lê 2 bytes (LSB, MSB)
    i2c_read_blocking(I2C_SENS, GY33_ADDR, b, 2, false);
    return (uint16_t)(((uint16_t)b[1] << 8) | b[0]);
}

static void gy33_init(void) {
    // Habilita o sensor (POWER ON + RGBC)
    gy33_write_u8(REG_ENABLE, 0x03);

    // Tempo de integração (ex.: 0xD5 ≈ ~100 ms)
    gy33_write_u8(REG_ATIME, 0xD5);

    // Ganho (0x00 = 1x)
    gy33_write_u8(REG_CONTROL, 0x00);
}

static void gy33_read_rgbc(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
    *c = gy33_read_u16(REG_CDATA);
    *r = gy33_read_u16(REG_RDATA);
    *g = gy33_read_u16(REG_GDATA);
    *b = gy33_read_u16(REG_BDATA);
}

// -------------------- Classificação simples da cor --------------------

static const char *nome_cor(uint16_t r, uint16_t g, uint16_t b) {
    // Vermelho intenso (muito dominante)
    if (r >= RED_INTENSE_RAW &&
        r > g + DOMINANCE_MARGIN_RAW &&
        r > b + DOMINANCE_MARGIN_RAW) {
        return "Vermelho (intenso)";
    }

    // Vermelho
    if (r >= RED_MIN_RAW &&
        r > g + DOMINANCE_MARGIN_RAW &&
        r > b + DOMINANCE_MARGIN_RAW) {
        return "Vermelho";
    }

    // Verde
    if (g >= GREEN_MIN_RAW &&
        g > r + DOMINANCE_MARGIN_RAW &&
        g > b + DOMINANCE_MARGIN_RAW) {
        return "Verde";
    }

    // Azul
    if (b >= BLUE_MIN_RAW &&
        b > r + DOMINANCE_MARGIN_RAW &&
        b > g + DOMINANCE_MARGIN_RAW) {
        return "Azul";
    }

    // Combinações básicas
    if (r >= RED_MIN_RAW && g >= GREEN_MIN_RAW && b < BLUE_MIN_RAW) {
        return "Amarelo";
    }
    if (r >= RED_MIN_RAW && b >= BLUE_MIN_RAW && g < GREEN_MIN_RAW) {
        return "Magenta";
    }
    if (g >= GREEN_MIN_RAW && b >= BLUE_MIN_RAW && r < RED_MIN_RAW) {
        return "Ciano";
    }

    return "Mista";
}

// -------------------- Função principal --------------------

int main() {
    // Inicializa GPIO do LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, LED_OFF);

    // Inicializa UART para debug (PicoProbe -> VSCode / PuTTY)
    stdio_uart_init_full(uart0, BAUD_RATE, UART_TX_PIN, UART_RX_PIN);

    // Pequeno delay para dar tempo de abrir o terminal
    sleep_ms(500);

    printf("=== Depuracao GY-33 (sensor de cor) - Embarcatech ===\n");

    // Inicializa I2C0 nos pinos 0 (SDA) e 1 (SCL)
    i2c_init(I2C_SENS, 400 * 1000); // 400 kHz
    gpio_set_function(I2C_SENS_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SENS_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SENS_SDA);
    gpio_pull_up(I2C_SENS_SCL);

    // Inicializa o sensor de cor
    gy33_init();
    printf("GY-33 inicializado.\n");

    // Variáveis para observar no debugger
    uint32_t amostra_id = 0;
    uint16_t r = 0, g = 0, b = 0, c = 0;
    const char *cor_atual = "N/A";

    while (true) {
        // Lê valores RGBC
        gy33_read_rgbc(&r, &g, &b, &c);

        // Classifica a cor
        cor_atual = nome_cor(r, g, b);
        amostra_id++;

        // Pisca o LED para indicar o loop
        gpio_put(LED_PIN, (amostra_id & 1) ? LED_ON : LED_OFF);

        // Saída para o terminal (depuração por printf)
        printf("Amostra %lu -> R=%u, G=%u, B=%u, C=%u | Cor: %s\n",
               (unsigned long)amostra_id,
               r, g, b, c,
               cor_atual);

        // Aqui é um ótimo ponto para colocar breakpoints no VSCode
        // e inspecionar r, g, b, c, cor_atual, amostra_id.

        sleep_ms(LOOP_DELAY_MS);
    }

    // Nunca chega aqui
    return 0;
}
