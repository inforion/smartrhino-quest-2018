#ifndef __WS2812B_H
#define __WS2812B_H

#include <stdint.h>
#include <stm32f0xx.h>

#define WS2812B_CORRECT_GAMMA      0

#define WS2812B_MAX_LED_COUNT      4

#define WS2812B_PWM_PERIOD         1.25  // us
#define WS2812B_PWM_PULSE_HIGH     0.90  // us
#define WS2812B_PWM_PULSE_LOW      0.35  // us
#define WS2812B_RESET_TIME         50.0  // us

#define WS2812B_RESET_COUNT        (uint32_t) (WS2812B_RESET_TIME / WS2812B_PWM_PERIOD)

#define WS2812B_PWM_DUTY_HIGH      WS2812B_PWM_PULSE_HIGH / WS2812B_PWM_PERIOD
#define WS2812B_PWM_DUTY_LOW       WS2812B_PWM_PULSE_LOW / WS2812B_PWM_PERIOD

#define WS2812B_TIMER_CLK_FREQ     (uint32_t) 48000000  // Hz
#define WS2812B_TIMER_PWM_FREQ     (uint32_t) (1000000 / WS2812B_PWM_PERIOD)  // Hz
#define WS2812B_TIMER_PERIOD       (uint32_t) ((WS2812B_TIMER_CLK_FREQ / WS2812B_TIMER_PWM_FREQ-1)/2)
#define WS2812B_TIMER_PSC          (uint32_t) (SystemCoreClock / WS2812B_TIMER_CLK_FREQ)
#define WS2812B_TIMER_PULSE_HIGH   (uint32_t) (WS2812B_TIMER_PERIOD * WS2812B_PWM_DUTY_HIGH + 1)
#define WS2812B_TIMER_PULSE_LOW    (uint32_t) (WS2812B_TIMER_PERIOD * WS2812B_PWM_DUTY_LOW + 1)

typedef struct {
    int16_t hue;
    uint8_t saturation;
    uint8_t value;
} hsv_t;

typedef struct {
    uint8_t green;
    uint8_t red;
    uint8_t blue;
} rgb_t;

// PWM timing struct
typedef struct {
    // order is important!
    uint8_t green[8];
    uint8_t red[8];
    uint8_t blue[8];
} pwm_led_t;


typedef struct {
    pwm_led_t leds[WS2812B_MAX_LED_COUNT];
    const uint8_t zeros[WS2812B_RESET_COUNT];
} pwm_chain_t;


typedef struct {
	volatile int busy;
	
    rgb_t rgb[WS2812B_MAX_LED_COUNT];
	uint8_t led_count;
    
    pwm_chain_t chain;
    
    int channel;
    TIM_HandleTypeDef* timer;
} ws2812b_t;

static inline void ws2812b_interrupt(ws2812b_t* ws2812b) {
    HAL_TIM_PWM_Stop_DMA(ws2812b->timer, ws2812b->channel);
    ws2812b->busy = 0;
}

int ws2812b_init(ws2812b_t* ws2812b, int led_count, TIM_HandleTypeDef* timer, int channel);

 int ws2812b_send(ws2812b_t* ws2812b);
void ws2812b_wait(ws2812b_t* ws2812b);

void ws2812b_send_wait(ws2812b_t* ws2812b);

int ws2812b_set_rgb(ws2812b_t* ws2812b, int index, uint8_t red, uint8_t green, uint8_t blue);
int ws2812b_set_hsv(ws2812b_t* ws2812b, int index, int16_t hue, uint8_t saturation, uint8_t value);
int ws2812b_send_rgb_wait(ws2812b_t* ws2812b, int index, uint8_t red, uint8_t green, uint8_t blue);
int ws2812b_send_hsv_wait(ws2812b_t* ws2812b, int index, int16_t hue, uint8_t saturation, uint8_t value);
void ws2812b_send_all_rgb_wait(ws2812b_t* ws2812b, uint8_t red, uint8_t green, uint8_t blue);
void ws2812b_send_all_hsv_wait(ws2812b_t* ws2812b, int16_t hue, uint8_t saturation, uint8_t value);
void ws2812b_disable_all_wait(ws2812b_t* ws2812b);

#endif //__WS2812B_H
