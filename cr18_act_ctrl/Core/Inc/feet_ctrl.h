/*
 * JerkCtrl.h
 *
 *  Created on: Jan 14, 2017
 *      Author: yusaku
 */

#ifndef INCLUDE_FEET_CTRL_H_
#define INCLUDE_FEET_CTRL_H_

#include "stm32f1xx_hal.h"
#include <cmath>

class FeetCtrl
{
private:
    struct StepperSegment
    {
        // remaining timestep to be executed for this segment
        int remaining_ticks = 0;

        // number of steps (in total) to be traveled in this segment
        // represented by its absolute value
        // dy in bresenham's algorithm
        int steps_total[3] = { 0, 0, 0 };

        // dx in bresenham's algorithm
        int ticks_total = 0;

        //int bresenham_error = 0;

        uint32_t dir_pattern = 0;

        int step_dir[3] = { 1, 1, 1 };
    };

    static constexpr double steps_per_rev = 16 * 200;
    static constexpr double steps_per_rad = steps_per_rev / (2.0 * M_PI);
    static constexpr double maximum_velocity = 6 * steps_per_rev;       // in [milli-step/ms]
    static constexpr double maximum_acceleration = steps_per_rev / 20; //32;//16 * 200 * 100 / 40 / 100;   // in [milli-step/ms^2]

    static constexpr int ticks_per_ms = 50;      //100;       // TIM4 50 kHz

    GPIO_TypeDef *m_step_gpio;
    volatile uint16_t m_step_pin[3];
    GPIO_TypeDef *m_dir_gpio;
    volatile uint16_t m_dir_pin[3];

    volatile int m_target_velocity[3] = { 0, 0, 0 };
    volatile int m_current_velocity[3] = { 0, 0, 0 };      // in milli-steps-per-milli-second aka step-per-second
    volatile int m_actual_velocity[3] = { 0, 0, 0 };
    //volatile bool m_is_idle = true;

    volatile double m_pos_trans_x = 0.0;
    volatile double m_pos_trans_y = 0.0;
    volatile double m_pos_rot_z = 0.0;

    // bresenham's algorithm: initial value : -dx
    volatile int m_bresenham_error[3] = { -ticks_per_ms * 1000, -ticks_per_ms * 1000, -ticks_per_ms * 1000 };

    volatile bool m_enabled = false;

    static constexpr int seg_buf_size = 8;                     // size of segment buffer
    static constexpr int seg_buf_mask = seg_buf_size - 1;       // mask
    StepperSegment seg_buf[seg_buf_size];                              // segment buffer
    volatile int seg_buf_head = 0;                                       // pointer for store
    volatile int seg_buf_tail = 0;                                       // pointer for load
    StepperSegment *current_segment = seg_buf; // pointer to current segment which is currently being executed

public:
    FeetCtrl(GPIO_TypeDef *step_gpio, uint16_t step_pin_x, uint16_t step_pin_y, uint16_t step_pin_z, GPIO_TypeDef *dir_gpio,
            uint16_t dir_pin_x, uint16_t dir_pin_y, uint16_t dir_pin_z);

    inline void disable(void)
    {
        this->m_target_velocity[0] = 0;
        this->m_target_velocity[1] = 0;
        this->m_target_velocity[2] = 0;
        this->m_enabled = false;
    }

    inline void enable(void)
    {
        this->m_enabled = true;
    }

    void calculate_profile(void);
    void control(void);

    // target: rad/s
    void set_target(float * const target);
    void set_target(float (&target)[3]);

    void reset_step(void)
    {
        m_step_gpio->BRR = m_step_pin[0] | m_step_pin[1] | m_step_pin[2];      // un-step
    }

    bool motion_completed(void)
    {
        return (m_current_velocity == m_target_velocity);
    }

    // workhorse
    // this needs to be called as often as ticks_per_sec.
    void tick(void)
    {
        /*
         if (!this->m_enabled)
         {
         return;
         }
         */

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

            //current_segment = &seg_buf[seg_buf_tail];
            //seg_buf_tail = ((seg_buf_tail + 1) & seg_buf_mask);

            current_segment = &seg_buf[seg_buf_tail++];
            seg_buf_tail &= seg_buf_mask;

            //m_actual_velocity = 0;
        }

        this->m_dir_gpio->BSRR = current_segment->dir_pattern;

        for (int i = 0; i < 3; i++)
        {
            if (this->m_bresenham_error[i] > 0)
            {
                m_step_gpio->BSRR = m_step_pin[i];     // step
                m_actual_velocity[i] += current_segment->step_dir[i];
                this->m_bresenham_error[i] -= 2 * current_segment->ticks_total;    // -2*dx
            }

            // 2*dy
            this->m_bresenham_error[i] += 2 * current_segment->steps_total[i];
        }

        current_segment->remaining_ticks--;
    }
};

#endif /* INCLUDE_FEET_CTRL_H_ */
