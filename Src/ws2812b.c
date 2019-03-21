#include <stdint.h>
#include <assert.h>
#include <string.h>
#include "ws2812b.h"

int ws2812b_send(ws2812b_t* ws2812b);

static inline uint8_t gamma_correction(uint8_t v) {
    #if (WS2812B_CORRECT_GAMMA == 1) 
        return (v * v + v) >> 8;
    #else
        return v;
    #endif
}

#define HUE(result, h) { \
        int16_t h1 = h % 360; \
        result = h1 < 0 ? 360 + h1 : h1; \
    }
    
void hsv2rgb(rgb_t* rgb, hsv_t* hsv) {
    int hue;
    
    if (!hsv->value) {
        rgb->red = rgb->green = rgb->blue = 0;
    } else if (!hsv->saturation) {
        rgb->red= rgb->green = rgb->blue = hsv->value;
    } else {
        HUE(hue, hsv->hue);

        int sector = hue / 60;
        int angle = sector & 1 ? 60 - hue % 60 : hue % 60;

        int high = hsv->value;
        int low = (255 - hsv->saturation) * high / 255;
        int middle = low + (high - low) * angle / 60;

        switch (sector) {
            case 0: // red -> yellow
                rgb->red = high;
                rgb->green = middle;
                rgb->blue = low;
                break;
            case 1: // yellow -> green
                rgb->red = middle;
                rgb->green = high;
                rgb->blue = low;
                break;
            case 2: // green -> cyan
                rgb->red = low;
                rgb->green = high;
                rgb->blue = middle;
                break;
            case 3: // cyan -> blue
                rgb->red = low;
                rgb->green = middle;
                rgb->blue = high;
                break;
            case 4: // blue -> magenta
                rgb->red = middle;
                rgb->green = low;
                rgb->blue = high;
                break;
            case 5: // magenta -> red
                rgb->red = high;
                rgb->green = low;
                rgb->blue = middle;
        }
    }
}

#define PWM_CONVERT_BIT(result, value, mask) { \
    result = value & mask ? WS2812B_TIMER_PULSE_HIGH : WS2812B_TIMER_PULSE_LOW; \
}

#define PWM_CONVERT_VALUE(result, value) { \
    PWM_CONVERT_BIT(result[0], value, 0x80); \
    PWM_CONVERT_BIT(result[1], value, 0x40); \
    PWM_CONVERT_BIT(result[2], value, 0x20); \
    PWM_CONVERT_BIT(result[3], value, 0x10); \
    PWM_CONVERT_BIT(result[4], value, 0x08); \
    PWM_CONVERT_BIT(result[5], value, 0x04); \
    PWM_CONVERT_BIT(result[6], value, 0x02); \
    PWM_CONVERT_BIT(result[7], value, 0x01); \
}

static inline void __ws2812b_clear_pwm_buf(ws2812b_t* ws2812b) {
    memset(&ws2812b->chain, 0, sizeof(ws2812b->chain));
}

static inline void __ws2812b_clear_rgb_buf(ws2812b_t* ws2812b) {
    memset(&ws2812b->rgb, 0, sizeof(ws2812b->rgb));
}

int ws2812b_set_rgb(ws2812b_t* ws2812b, int index, uint8_t red, uint8_t green, uint8_t blue) {
//	printf("ws2812b set rgb: index=%d r=%d g=%d b=%d\r\n", index, red, green, blue);
    if (index >= ws2812b->led_count) {
        return -2;
    }
    
    ws2812b->rgb[index].red = gamma_correction(red);
    ws2812b->rgb[index].green = gamma_correction(green);
    ws2812b->rgb[index].blue = gamma_correction(blue);
    
    return 0;
}    

int ws2812b_set_hsv(ws2812b_t* ws2812b, int index, int16_t hue, uint8_t saturation, uint8_t value) {
//	printf("ws2812b set hsv: index=%d h=%d s=%d v=%d\r\n", index, hue, saturation, value);
    hsv_t hsv = { .hue = hue, .saturation = saturation, .value = value };
    rgb_t rgb;
    
    hsv2rgb(&rgb, &hsv);
    
    return ws2812b_set_rgb(ws2812b, index, rgb.red, rgb.green, rgb.blue);
}

int ws2812b_send_rgb_wait(ws2812b_t* ws2812b, int index, uint8_t red, uint8_t green, uint8_t blue) {
	if (ws2812b_set_rgb(ws2812b, index, red, green, blue) < 0) {
		return -1;
	}
	ws2812b_send_wait(ws2812b);
	return 0;
}

int ws2812b_send_hsv_wait(ws2812b_t* ws2812b, int index, int16_t hue, uint8_t saturation, uint8_t value) {
	if (ws2812b_set_hsv(ws2812b, index, hue, saturation, value) < 0) {
		return -1;
	}
	ws2812b_send_wait(ws2812b);
	return 0;
}

void ws2812b_send_all_rgb_wait(ws2812b_t* ws2812b, uint8_t red, uint8_t green, uint8_t blue) {
	for (uint8_t k = 0; k < ws2812b->led_count; k++)
		ws2812b_set_rgb(ws2812b, k, red, green, blue);
	ws2812b_send_wait(ws2812b);
}

void ws2812b_send_all_hsv_wait(ws2812b_t* ws2812b, int16_t hue, uint8_t saturation, uint8_t value) {
	for (uint8_t k = 0; k < ws2812b->led_count; k++)
		ws2812b_set_hsv(ws2812b, k, hue, saturation, value);
	ws2812b_send_wait(ws2812b);
}

void ws2812b_disable_all_wait(ws2812b_t* ws2812b) {
	ws2812b_send_all_rgb_wait(ws2812b, 0, 0, 0);
}

void ws2812b_wait(ws2812b_t* ws2812b) {
//	printf("ws2812b wait while busy...\r\n");
    while (ws2812b->busy);
}

void ws2812b_send_wait(ws2812b_t* ws2812b) {
    ws2812b_wait(ws2812b);
    ws2812b_send(ws2812b);
}

int ws2812b_send(ws2812b_t* ws2812b) {
    if (ws2812b->busy) {
//        printf("ws2812b send: DMA is busy now!\r\n");
        return -1;
    }
    
    ws2812b->busy = 1;
    
    for (int k = 0; k < ws2812b->led_count; k++) {
        PWM_CONVERT_VALUE(ws2812b->chain.leds[k].red, ws2812b->rgb[k].red);
        PWM_CONVERT_VALUE(ws2812b->chain.leds[k].green, ws2812b->rgb[k].green);
        PWM_CONVERT_VALUE(ws2812b->chain.leds[k].blue, ws2812b->rgb[k].blue);
    }

    int dma_count = sizeof(pwm_led_t) * ws2812b->led_count + WS2812B_RESET_COUNT;
    
//    printf("ws2812b send: start at ticks=%lu count=%d\r\n", HAL_GetTick(), dma_count);

    HAL_TIM_PWM_Start_DMA(
        ws2812b->timer, 
        ws2812b->channel, 
        (void*) &ws2812b->chain, 
        dma_count);
    
//    printf("ws2812b send: done\r\n");

    return 0;
}

int ws2812b_init(ws2812b_t* ws2812b, int led_count, TIM_HandleTypeDef* timer, int channel) {
//    printf("ws2812b driver init: start initialization count=%d timer=%08lX ch=%d reset=%lu\r\n",
//        led_count, (uint32_t) timer, channel, WS2812B_RESET_COUNT);

    if (led_count > WS2812B_MAX_LED_COUNT) {
//        printf("ws2812b driver init: too much led count!\r\n");
        return -1;
    }
    
    if (timer->State != HAL_TIM_STATE_READY) {
//        printf("ws2812b driver init: PWM timer must be initialized!\r\n");
        return -2;
    }
    
    // reinit timer parameters
    timer->Init.Prescaler = WS2812B_TIMER_PSC;
    timer->Init.Period = WS2812B_TIMER_PERIOD;
    if (HAL_OK != HAL_TIM_PWM_Init(timer)) {
//        printf("ws2812b driver init: can't configure PWM timer!\r\n");
        return -3;
    }
    
//    printf("ws2812b driver init: psc=%lu period=%lu high=%lu low=%lu timerclk=%lu sysclk=%lu\r\n",
//        WS2812B_TIMER_PSC, WS2812B_TIMER_PERIOD,
//        WS2812B_TIMER_PULSE_HIGH, WS2812B_TIMER_PULSE_LOW,
//        WS2812B_TIMER_CLK_FREQ, SystemCoreClock);
    
    ws2812b->busy = 0;
    
    ws2812b->led_count = led_count;

    __ws2812b_clear_rgb_buf(ws2812b);
    __ws2812b_clear_pwm_buf(ws2812b);
    
    ws2812b->channel = channel;
    ws2812b->timer = timer;

//    int TIM_DMA_Handle_index = 0;
//    switch (channel) {
//		case TIM_CHANNEL_1:
//			TIM_DMA_Handle_index = TIM_DMA_ID_CC1;
//			break;
//		case TIM_CHANNEL_2:
//			TIM_DMA_Handle_index = TIM_DMA_ID_CC2;
//			break;
//		case TIM_CHANNEL_3:
//			TIM_DMA_Handle_index = TIM_DMA_ID_CC3;
//			break;
//		case TIM_CHANNEL_4:
//			TIM_DMA_Handle_index = TIM_DMA_ID_CC4;
//			break;
//		default:
//			return -4;
//    }

//    HAL_DMA_RegisterCallback(timer->hdma[TIM_DMA_Handle_index], HAL_DMA_XFER_CPLT_CB_ID, ws2812b_interrupt);
        
//    printf("ws2812b driver init: ok\r\n");
    
    return 0;
}
