/*
 * JerkCtrl.cpp
 *
 *  Created on: Jan 14, 2017
 *      Author: yusaku
 */

#include "stepper_velocity_ctrl.h"

#include "stm32f1xx_hal.h"
#include <cmath>
#include <cstdlib>

StepperVelocityCtrl::StepperVelocityCtrl(GPIO_TypeDef *step_gpio, uint16_t step_pin, GPIO_TypeDef *dir_gpio, uint16_t dir_pin)
{
    m_step_gpio = step_gpio;
    m_step_pin = step_pin;
    m_dir_gpio = dir_gpio;
    m_dir_pin = dir_pin;

    seg_buf_head = 0;
    seg_buf_tail = 0;

    for (int i = 0; i < seg_buf_size; i++)
    {
        seg_buf[i].steps_total = 0;
        seg_buf[i].remaining_ticks = 0;
        seg_buf[i].ticks_total = 0;
    }
}

void StepperVelocityCtrl::calculate_profile(void)
{
    //

    if (!this->m_enabled)
    {
        this->m_target_velocity = 0;
    }

    int vel_diff = m_target_velocity - m_current_velocity;
    int vel_diff_abs = abs(vel_diff);
    int accel_until_ms = vel_diff_abs / maximum_acceleration;  // in ms

    int accel = 0;

    if (vel_diff < 0)
    {
        accel = -maximum_acceleration;
    }
    else
    {
        accel = maximum_acceleration;
    }

    while (((seg_buf_head + 1) & seg_buf_mask) != seg_buf_tail)
    {
        // fill the queue

        auto *seg = &seg_buf[seg_buf_head];

        if (accel_until_ms > 0)
        {
            m_current_velocity += accel;
            accel_until_ms--;
        }
        else
        {
            m_current_velocity = m_target_velocity;
        }

        int velocity_sgn = 1;
        int velocity_abs;

        if (m_current_velocity < 0)
        {
            seg->dir_pattern = this->m_dir_pin << 16;
            velocity_sgn = -1;
        }
        else
        {
            seg->dir_pattern = this->m_dir_pin;
            velocity_sgn = 1;
        }

        seg->step_dir = velocity_sgn;
        velocity_abs = velocity_sgn * m_current_velocity;

        seg->steps_total = velocity_abs;

        seg->ticks_total = ticks_per_ms * 1000;
        seg->remaining_ticks = ticks_per_ms;

        seg_buf_head = ((seg_buf_head + 1) & seg_buf_mask);

        //trace_printf("%d,%d,%d\n", m_target, m_current_position, m_current_velocity);
    }
}

// target: rad/s
void StepperVelocityCtrl::set_target(float target)
{
    if (!this->m_enabled)
    {
        this->m_target_velocity = 0;
    }
    else
    {
        m_target_velocity = target * steps_per_rad;

        if (m_target_velocity < -maximum_velocity)
        {
            m_target_velocity = -maximum_velocity;
        }
        else if (m_target_velocity > maximum_velocity)
        {
            m_target_velocity = maximum_velocity;
        }
    }

    //calculate_profile();

    //m_is_idle = false;
}

