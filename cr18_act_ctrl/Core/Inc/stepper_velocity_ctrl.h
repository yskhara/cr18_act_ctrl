/*
 * JerkCtrl.h
 *
 *  Created on: Jan 14, 2017
 *      Author: yusaku
 */

#ifndef INCLUDE_STEPPER_VELOCITY_CTRL_H_
#define INCLUDE_STEPPER_VELOCITY_CTRL_H_

#include "stm32f1xx_hal.h"
#include "stepper_segment.h"
#include <cmath>

class StepperVelocityCtrl
{
private:
    static constexpr double steps_per_rev = 16 * 200;
    static constexpr double steps_per_rad = steps_per_rev / (2.0 * M_PI);
    static constexpr double maximum_velocity = 6 * steps_per_rev;       // in [milli-step/ms]
    static constexpr double maximum_acceleration = steps_per_rev / 20;      //32;//16 * 200 * 100 / 40 / 100;   // in [milli-step/ms^2]

    static constexpr int ticks_per_ms = 50;//100;       // TIM4 50 kHz

    GPIO_TypeDef *m_step_gpio;
    GPIO_TypeDef *m_dir_gpio;
    volatile uint16_t m_step_pin;
    volatile uint16_t m_dir_pin;

    volatile int m_target_velocity = 0;
    volatile int m_current_velocity = 0;      // in milli-steps-per-milli-second aka step-per-second
    //volatile int m_actual_velocity = 0;
    //volatile bool m_is_idle = true;

    // bresenham's algorithm: initial value : -dx
    volatile int m_bresenham_error = -ticks_per_ms * 1000;

    static constexpr int seg_buf_size = 8;                     // size of segment buffer
    static constexpr int seg_buf_mask = seg_buf_size - 1;       // mask
    StepperSegment seg_buf[seg_buf_size];                              // segment buffer
    volatile int seg_buf_head = 0;                                       // pointer for store
    volatile int seg_buf_tail = 0;                                       // pointer for load
    StepperSegment *current_segment = seg_buf; // pointer to current segment which is currently being executed

public:
    StepperVelocityCtrl(GPIO_TypeDef *step_gpio, uint16_t step_pin, GPIO_TypeDef *dir_gpio, uint16_t dir_pin);

    void reset_position(void);
    void calculate_profile(void);
    void control(void);

    // target: rad/s
    void set_target(float target);

    void reset_step(void)
    {
        m_step_gpio->BRR = m_step_pin;      // un-step
    }

    bool motion_completed(void)
    {
        return (m_current_velocity == m_target_velocity);
    }

    // workhorse
    // this needs to be called as often as ticks_per_sec.
    void tick(void)
    {
        // load next segment if neccesary
        if (current_segment->remaining_ticks <= 0)
        {
            // segment complete; try to load next segment
            // check whether the queue is empty
            if (seg_buf_head == seg_buf_tail)
            {
                // next queue is also empty; starving: panic
                return;
            }

            current_segment = &seg_buf[seg_buf_tail];
            seg_buf_tail = ((seg_buf_tail + 1) & seg_buf_mask);


            //m_actual_velocity = 0;
        }

        this->m_dir_gpio->BSRR = current_segment->dir_pattern;

        if (this->m_bresenham_error > 0)
        {
            m_step_gpio->BSRR = m_step_pin;     // step

            //m_actual_velocity += current_segment->step_dir;

            // -2*dx
            this->m_bresenham_error -= 2 * current_segment->ticks_total;
        }

        // 2*dy
        this->m_bresenham_error += 2 * current_segment->steps_total;

        current_segment->remaining_ticks--;
    }
};

#endif /* INCLUDE_STEPPER_VELOCITY_CTRL_H_ */
