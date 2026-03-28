//
// Created by fish on 2024/9/2.
//

#include "bsp_led.h"
#include "bsp_def.h"
#include "spi.h"

#define WS2812_HIGH    0xf0
#define WS2812_LOW     0xc0
#define WS2812_PORT    &hspi6

static uint8_t buf[24];

void bsp_led_set(uint8_t r, uint8_t g, uint8_t b) {
    for(int i = 0; i < 8; i++) {
        buf[7-i]  = (((g >> i) & 1) ? WS2812_HIGH : WS2812_LOW) >> 1;
        buf[15-i] = (((r >> i) & 1) ? WS2812_HIGH : WS2812_LOW) >> 1;
        buf[23-i] = (((b >> i) & 1) ? WS2812_HIGH : WS2812_LOW) >> 1;
    }
    HAL_SPI_Transmit(WS2812_PORT, buf, 24, HAL_MAX_DELAY);
}

void bsp_led_set_hsv(float h, float s, float v) {
    h = fmodf(h, 1.0f) * 6.0f;

    float f = h - floorf(h), p = v * (1 - s), q = v * (1 - s * f), t = v * (1 - s * (1 - f));

    float r,g,b;
    switch((int) (floorf(h)) % 6){
    case 0: r=v; g=t; b=p; break;
    case 1: r=q; g=v; b=p; break;
    case 2: r=p; g=v; b=t; break;
    case 3: r=p; g=q; b=v; break;
    case 4: r=t; g=p; b=v; break;
    default:r=v; g=p; b=q; break;
    }

    bsp_led_set((uint8_t) (r*255+0.5f), (uint8_t) (g*255+0.5f), (uint8_t) (b*255+0.5f));
}