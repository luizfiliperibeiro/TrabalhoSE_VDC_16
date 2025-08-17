#ifndef WS2812_H
#define WS2812_H

#include "hardware/pio.h"

typedef struct {
    PIO pio;
    uint sm;
    uint offset;
    uint pin;
    uint length;
} ws2812_t;

void ws2812_init(ws2812_t *ws, uint pin, uint length);
void ws2812_set_pixel(ws2812_t *ws, uint pixel, uint8_t r, uint8_t g, uint8_t b);
void ws2812_clear(ws2812_t *ws);
void ws2812_show(ws2812_t *ws);

#endif