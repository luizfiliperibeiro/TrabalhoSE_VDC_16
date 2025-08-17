#include "ws2812.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "ws2812.pio.h" // inclua corretamente o header gerado!

void ws2812_init(ws2812_t *ws, uint pin, uint length) {
    ws->pin = pin;
    ws->length = length;
    ws->pio = pio0; // Use o PIO 0 (pode trocar para pio1 se quiser)
    ws->sm = pio_claim_unused_sm(ws->pio, true); // Pega uma state machine livre

    // Adiciona o programa no PIO e salva o offset
    ws->offset = pio_add_program(ws->pio, &ws2812_program);

    // Agora inicializa o programa, passando todos os argumentos necessários
    ws2812_program_init(ws->pio, ws->sm, ws->offset, pin, 800000, false);

    // Inicialmente apaga os LEDs
    ws2812_clear(ws);
}

void ws2812_set_pixel(ws2812_t *ws, uint pixel, uint8_t r, uint8_t g, uint8_t b) {
    if (pixel >= ws->length) return;

    uint32_t color = ((uint32_t)(g) << 16) | ((uint32_t)(r) << 8) | (uint32_t)(b);
    pio_sm_put_blocking(ws->pio, ws->sm, color << 8u); // envia cor formatada
}

void ws2812_clear(ws2812_t *ws) {
    for (uint i = 0; i < ws->length; i++) {
        ws2812_set_pixel(ws, i, 0, 0, 0); // seta todos os LEDs para preto
    }
    ws2812_show(ws);
}

void ws2812_show(ws2812_t *ws) {
    sleep_ms(1); // Espera os dados serem transmitidos completamente
}