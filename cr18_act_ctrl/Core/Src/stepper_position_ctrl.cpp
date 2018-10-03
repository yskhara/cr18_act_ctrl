/*
 * JerkCtrl.cpp
 *
 *  Created on: Jan 14, 2017
 *      Author: yusaku
 */

#include "stepper_position_ctrl.h"
#include <cmath>
#include <cstdlib>

StepperPositionCtrl::StepperPositionCtrl(GPIO_TypeDef *step_gpio, uint16_t step_pin, GPIO_TypeDef *dir_gpio, uint16_t dir_pin)
{
    m_step_gpio = step_gpio;
    m_step_pin = step_pin;
    m_dir_gpio = dir_gpio;
    m_dir_pin = dir_pin;

    seg_buf_head = 0;
    seg_buf_tail = 0;

    for (int j = 0; j < seg_buf_size; j++)
    {
        //seg_buf[j].bresenham_error = 0;
        seg_buf[j].remaining_ticks = 0;
        seg_buf[j].steps_total = 0;
        seg_buf[j].ticks_total = 0;
    }
}

void StepperPositionCtrl::reset_position(void)
{
    m_current_velocity = 0;
    m_current_position = 0;
    m_actual_position = 0;
    m_target_position = 0;
}

void StepperPositionCtrl::calculate_profile(void)
{
    if (!this->m_enabled)
    {
        this->m_current_velocity = 0;
        this->m_current_position = 0;
        this->m_actual_position = 0;
        this->m_target_position = 0;
    }

    int accel_until_ms = 0;
    int cruise_until_ms = 0;
    int decel_until_ms = 0;
    int cruise_vel = 0;

    int target = m_target_position;

    int position_delta = m_target_position - m_current_position;
    int position_delta_sgn = 1;
    if (position_delta < 0)
    {
        position_delta_sgn = -1;
    }
    int position_delta_abs = position_delta_sgn * position_delta;

    int decision_delta = (int) (((2 * maximum_velocity * maximum_velocity) - (m_current_velocity * m_current_velocity))
            / (2.0 * maximum_acceleration) + 0.9);

    cruise_until_ms = (position_delta_abs - decision_delta) / maximum_velocity;

    int accel = position_delta_sgn * maximum_acceleration;
    int decel = -accel;

    if (cruise_until_ms > 0)
    {
        // trapezoidal profile
        accel_until_ms = (maximum_velocity - (position_delta_sgn * m_current_velocity)) / maximum_acceleration;
        decel_until_ms = maximum_velocity / maximum_acceleration;
        cruise_vel = position_delta_sgn * maximum_velocity;
    }
    else
    {
        int cruise_vel_abs = (int) sqrt(
                ((2.0 * position_delta_abs * maximum_acceleration) + ((double) m_current_velocity * m_current_velocity)) / 2.0);
        cruise_vel = position_delta_sgn * cruise_vel_abs;

        int current_vel_abs = abs(m_current_velocity);

        if (m_current_velocity * cruise_vel < 0)
        {
            // over-shoot profile
            //accel = position_delta_sgn * maximum_acceleration;
            //decel = -accel;
            accel_until_ms = (current_vel_abs + cruise_vel_abs) / maximum_acceleration;
            decel_until_ms = cruise_vel_abs / maximum_acceleration;
        }
        else if (current_vel_abs > cruise_vel_abs)
        {
            // special consideration for double-deceleration profile
            accel = -position_delta_sgn * maximum_acceleration;
            decel = accel;

            accel_until_ms = (current_vel_abs - cruise_vel_abs) / maximum_acceleration;
            decel_until_ms = cruise_vel_abs / maximum_acceleration;
        }
        else if (cruise_vel_abs > 500)
        {
            // triangle profile
            accel_until_ms = (cruise_vel_abs - current_vel_abs) / maximum_acceleration;
            decel_until_ms = cruise_vel_abs / maximum_acceleration;
        }
        else
        {
            // no time
            accel_until_ms = 0;
            decel_until_ms = 0;
        }
        //decel_until_ms = cruise_vel_abs / maximum_acceleration;
        cruise_until_ms = 0;
    }

    while (((seg_buf_head + 1) & seg_buf_mask) != seg_buf_tail)
    {
        // fill the queue

        if (accel_until_ms > 0)
        {
            // acceleration phase
            m_current_velocity += accel;
            accel_until_ms--;
        }
        else if (cruise_until_ms > 0)
        {
            // cruise phase
            m_current_velocity = cruise_vel;
            cruise_until_ms--;
        }
        else if (decel_until_ms > 1)
        {
            // decel phase
            m_current_velocity += decel;
            decel_until_ms--;
        }
        else
        {
            // end of motion
            m_current_velocity = target - m_current_position;
        }

        auto *seg = &seg_buf[seg_buf_head];
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

        //volatile int steps = (int) ((velocity_abs / 1000.0) + 0.5);  // in steps
        //int steps = (int) (velocity_abs / 1000);  // in steps
        seg->steps_total = velocity_abs;
        seg->ticks_total = ticks_per_ms * 1000;
        seg->remaining_ticks = ticks_per_ms;

        seg->step_dir = velocity_sgn;

        seg_buf_head = ((seg_buf_head + 1) & seg_buf_mask);

        m_current_position += velocity_sgn * velocity_abs;
        //m_current_position += velocity_sgn * steps * 1000;

        //m_current_position = tmp_pos;
        //m_current_velocity = tmp_vel;

        //trace_printf("%d,%d,%d\n", m_target, m_current_position, m_current_velocity);
    }
}

void StepperPositionCtrl::set_target_position(int target)
{
    if (this->m_enabled)
    {
        this->m_target_position = target * 1000;       // convert to milli-steps
    }

    //calculate_profile();
}

