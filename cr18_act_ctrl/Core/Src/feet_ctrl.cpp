/*
 * JerkCtrl.cpp
 *
 *  Created on: Jan 14, 2017
 *      Author: yusaku
 */

#include "feet_ctrl.h"

#include "stm32f1xx_hal.h"
#include <cmath>
#include <cstdlib>

FeetCtrl::FeetCtrl(GPIO_TypeDef *step_gpio, uint16_t step_pin_x, uint16_t step_pin_y, uint16_t step_pin_z, GPIO_TypeDef *dir_gpio,
        uint16_t dir_pin_x, uint16_t dir_pin_y, uint16_t dir_pin_z)
        : m_step_gpio(step_gpio), m_step_pin { step_pin_x, step_pin_y, step_pin_z }, m_dir_gpio(dir_gpio), m_dir_pin { dir_pin_x,
                dir_pin_y, dir_pin_z }
{
    //m_step_gpio = step_gpio;
    //m_step_pin[0] = step_pin_x;
    //m_dir_gpio = dir_gpio;
    //m_dir_pin = dir_pin;

    seg_buf_head = 0;
    seg_buf_tail = 0;

    for (int i = 0; i < seg_buf_size; i++)
    {
        seg_buf[i].steps_total[0] = 0;
        seg_buf[i].steps_total[1] = 0;
        seg_buf[i].steps_total[2] = 0;
        seg_buf[i].remaining_ticks = 0;
        seg_buf[i].ticks_total = 0;
    }
}

void FeetCtrl::calculate_profile(void)
{
    if (!this->m_enabled)
    {
        this->m_target_velocity[0] = 0;
        this->m_target_velocity[1] = 0;
        this->m_target_velocity[2] = 0;
    }

#ifdef ACCEL_LIMIT
    int accel_until_ms = 0;
    int accel[3];

    int vel_diff[3];

    for (int i = 0; i < 3; i++)
    {
        vel_diff[i] = m_target_velocity[i] - m_current_velocity[i];
        int accel_until_ms_cand = abs(vel_diff[i]) / maximum_acceleration;  // in ms
        if (accel_until_ms_cand > accel_until_ms)
        {
            accel_until_ms = accel_until_ms_cand;
        }
    }

    for (int i = 0; i < 3; i++)
    {
        accel[i] = vel_diff[i] / accel_until_ms;
    }

#else

#endif

    while (((seg_buf_head + 1) & seg_buf_mask) != seg_buf_tail)
    {
        // fill the queue

        auto *seg = &seg_buf[seg_buf_head];

        seg->dir_pattern = 0;

        for (int i = 0; i < 3; i++)
        {

#ifdef ACCEL_LIMIT
            if (accel_until_ms > 0)
            {
                m_current_velocity[i] += accel[i];
                accel_until_ms--;
            }
            else
            {
                m_current_velocity[i] = m_target_velocity[i];
            }
#else
            m_current_velocity[i] = m_target_velocity[i];
#endif

            if (m_current_velocity[i] < 0)
            {
                seg->dir_pattern |= this->m_dir_pin[i] << 16;
                seg->step_dir[i] = -1;
            }
            else
            {
                seg->dir_pattern |= this->m_dir_pin[i];
                seg->step_dir[i] = 1;
            }

            // abs
            seg->steps_total[i] = seg->step_dir[i] * m_current_velocity[i];
        }

        seg->ticks_total = ticks_per_ms * 1000;
        seg->remaining_ticks = ticks_per_ms;

        seg_buf_head = ((seg_buf_head + 1) & seg_buf_mask);

        //trace_printf("%d,%d,%d\n", m_target, m_current_position, m_current_velocity);
    }
}

// target: rad/s
void FeetCtrl::set_target(float (&target)[3])
{
    this->set_target((float * const ) target);
}

void FeetCtrl::set_target(float * const target)
{
    if (!this->m_enabled)
    {
        this->m_target_velocity[0] = 0;
        this->m_target_velocity[1] = 0;
        this->m_target_velocity[2] = 0;
    }
    else
    {
        for (int i = 0; i < 3; i++)
        {
            m_target_velocity[i] = target[i] * steps_per_rad;

            if (m_target_velocity[i] < -maximum_velocity)
            {
                m_target_velocity[i] = -maximum_velocity;
            }
            else if (m_target_velocity[i] > maximum_velocity)
            {
                m_target_velocity[i] = maximum_velocity;
            }
        }
    }

    //calculate_profile();

    //m_is_idle = false;
}

