/*
 * JerkCtrl.h
 *
 *  Created on: Jan 14, 2017
 *      Author: yusaku
 */

#ifndef INCLUDE_STEPPER_SEGMENT_H_
#define INCLUDE_STEPPER_SEGMENT_H_

#include "stm32f1xx_hal.h"

struct StepperSegment
{
    // remaining timestep to be executed for this segment
    int remaining_ticks = 0;

    // number of steps (in total) to be traveled in this segment
    // represented by its absolute value
    // dy in bresenham's algorithm
    int steps_total = 0;

    // dx in bresenham's algorithm
    int ticks_total = 0;

    //int bresenham_error = 0;

    uint32_t dir_pattern = 0;

    int step_dir = 1;
};

#endif /* INCLUDE_STEPPER_SEGMENT_H_ */

