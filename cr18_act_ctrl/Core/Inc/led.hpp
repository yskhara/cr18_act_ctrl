/*
 * led.hpp
 *
 *  Created on: Oct 26, 2019
 *      Author: yusaku
 */

#ifndef INC_LED_HPP_
#define INC_LED_HPP_

#include "main.h"
#include "stm32f1xx_hal.h"

struct Color
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
};

constexpr Color __LED_COLOR_BLUE = { 0, 0, 255 };
constexpr Color __LED_COLOR_YELLOW = {255, 150,0};
constexpr Color __LED_COLOR_RED = {255,0,0};

const Color * const led_blue = &__LED_COLOR_BLUE;
const Color * const led_yellow = &__LED_COLOR_YELLOW;
const Color * const led_red = &__LED_COLOR_RED;


void init_led(void);
void led_process(void);
void led_set_color(const Color * const color);
void led_set_pattern(const uint8_t * pattern);

const uint8_t led_pattern_sin_1hz[51] =
{ 50, 0, 1, 4, 9, 16, 24, 35, 46, 59, 73, 88, 104, 119, 136, 151, 167, 182, 196, 209, 220, 231, 239, 246, 251, 254, 255, 254, 251, 246, 239, 231, 220,
        209, 196, 182, 167, 151, 136, 119, 104, 88, 73, 59, 46, 35, 24, 16, 9, 4, 1, };

const uint8_t led_pattern_exp_1hz[51] =
{ 50, 51, 51, 51, 51, 51, 51, 51, 51, 52, 52, 53, 55, 58, 62, 69, 79, 91, 108, 128, 150, 175, 199, 221, 239, 251, 255, 251, 239, 221, 199, 175, 150,
        128, 108, 91, 79, 69, 62, 58, 55, 53, 52, 52, 51, 51, 51, 51, 51, 51, 51, };

const uint8_t led_pattern_exp_0_5hz[101] =
{ 100, 26, 27, 28, 29, 30, 32, 35, 38, 43, 49, 57, 66, 77, 89, 104, 120, 137, 155, 174, 192, 209, 225, 237, 247, 253, 255, 253, 247, 237, 225, 209,
        192, 174, 155, 137, 120, 104, 89, 77, 66, 57, 49, 43, 38, 35, 32, 30, 29, 28, 27, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26,
        26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26 };

const uint8_t led_pattern_square_2hz[36] =
{ 35, 0, 32, 64, 96, 128, 160, 192, 224, 255, 255, 255, 255, 255, 255, 255, 255, 224, 192, 160, 128, 96, 64, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

const uint8_t led_pattern_square_128bpm[25] =
{ 24, 26, 27, 29, 34, 42, 57, 78, 108, 145, 184, 220, 246, 255, 246, 220, 184, 145, 108, 78, 57, 42, 34, 29, 27 };

#endif /* INC_LED_HPP_ */
