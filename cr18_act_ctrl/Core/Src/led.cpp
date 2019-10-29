/*
 * led.cpp
 *
 *  Created on: Oct 26, 2019
 *      Author: yusaku
 */

#include "led.hpp"

enum class led_pattern
    : uint8_t
    {
        Pattern_A
};

struct Color
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
};

constexpr Color BLUE =
{ 0, 0, 255 };

constexpr Color YELLOW = {255, 150,0};

uint8_t ms_per_frame = 20;
uint32_t start_time = 0;

uint8_t pattern_current_index = 0;
const uint8_t * current_pattern = led_pattern_square_128bpm;
const Color * current_color = &YELLOW;

void init_led(void)
{
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
    TIM1->BDTR |= TIM_BDTR_MOE;
}

void apply_color(const Color * color, uint8_t mul = 255)
{
    TIM1->CCR1 = color->R * 1000 * mul / (255 * 255);
    TIM1->CCR2 = color->G * 1000 * mul / (255 * 255);
    TIM1->CCR3 = color->B * 1000 * mul / (255 * 255);
}

void led_process(void)
{
    if (current_pattern[0] < pattern_current_index)
    {
        pattern_current_index = 1;
    }

    apply_color(current_color, current_pattern[pattern_current_index++]);
}

