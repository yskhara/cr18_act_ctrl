/*
 * JerkCtrl.h
 *
 *  Created on: Jan 14, 2017
 *      Author: yusaku
 */

#ifndef INCLUDE_STEPPER_POSITION_CTRL_H_
#define INCLUDE_STEPPER_POSITION_CTRL_H_

#include "stm32f1xx_hal.h"
#include "stepper_segment.h"

class StepperPositionCtrl
{
private:

    static constexpr double steps_per_mm = 16 * 200 * 3 / 40.0;
    //static constexpr int maximum_velocity =  125 * steps_per_mm;      // in [milli-step/ms]
    static constexpr int maximum_velocity = 10 * 3200;     // in [milli-step/ms]

    static constexpr int maximum_acceleration = 24; //16 * 200 * 100 / 40 / 100;    // in [milli-step/ms^2]
    //static constexpr int maximum_acceleration = 500;//32;//16 * 200 * 100 / 40 / 100;    // in [milli-step/ms^2]

    static constexpr int ticks_per_ms = 50;    //100;       // TIM4 50 kHz

    GPIO_TypeDef *m_step_gpio;
    GPIO_TypeDef *m_dir_gpio;
    volatile uint16_t m_step_pin;
    volatile uint16_t m_dir_pin;
    //uint32_t m_dir_pattern;

    volatile int m_target_position = 0;
    volatile int m_current_position = 0;         // in milli-steps
    volatile int m_actual_position = 0;
    volatile int m_current_velocity = 0;      // in milli-steps-per-milli-second
    //int m_step_dir = 1;                 // -1 for negative direction

    // bresenham's algorithm: initial value : -dx
    volatile int m_bresenham_error = -ticks_per_ms * 1000;

    volatile bool m_enabled = false;

    //volatile bool m_is_idle = true;
    //bool m_busy = false;

    static constexpr int seg_buf_size = 30;            // size of segment buffer
    static constexpr int seg_buf_mask = seg_buf_size - 1;       // mask
    StepperSegment seg_buf[seg_buf_size];                             // segment buffer
    volatile int seg_buf_head = 0;                                   // pointer for store
    volatile int seg_buf_tail = 0;                                    // pointer for load
    StepperSegment *current_segment = seg_buf; // pointer to current segment which is currently being executed

public:
    StepperPositionCtrl(GPIO_TypeDef *step_gpio, uint16_t step_pin, GPIO_TypeDef *dir_gpio, uint16_t dir_pin);

    void reset_position(void);

    inline void disable(void)
    {
        this->m_current_velocity = 0;
        this->m_current_position = 0;
        this->m_actual_position = 0;
        this->m_target_position = 0;

        this->m_enabled = false;
    }

    inline void enable(void)
    {
        this->m_enabled = true;
    }

    void calculate_profile(void);
    void set_target_position(int target);

    inline bool motion_completed(void)
    {
        return (m_actual_position * 1000 == m_target_position);
    }

    inline void reset_step(void)
    {
        this->m_step_gpio->BSRR = static_cast<uint32_t>(this->m_step_pin << 16);      // un-step
    }

    // workhorse
    // this needs to be called as often as ticks_per_sec.
    inline void tick(void)
    {
        // load next segment if neccesary
        if (current_segment->remaining_ticks <= 0)
        {
            // segment complete; try to load next segment
            // check whether the queue is empty
            if (seg_buf_head == seg_buf_tail)
            {
                // the queue is empty; starving: panic
                //m_is_idle = true;
                return;
            }

            current_segment = &seg_buf[seg_buf_tail++];
            seg_buf_tail &= seg_buf_mask;
        }

        this->m_dir_gpio->BSRR = current_segment->dir_pattern;

        if (this->m_bresenham_error > 0)
        {
            m_step_gpio->BSRR = m_step_pin;     // step

            m_actual_position += current_segment->step_dir;

            // -2*dx
            this->m_bresenham_error -= 2 * current_segment->ticks_total;
        }

        // 2*dy
        this->m_bresenham_error += 2 * current_segment->steps_total;

        current_segment->remaining_ticks--;
    }
};

#endif /* INCLUDE_STEPPER_POSITION_CTRL_H_ */

