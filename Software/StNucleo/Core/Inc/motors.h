/*
 * motors.h
 *
 *  Created on: 2019. júl. 17.
 *      Author: mm
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>


#if defined(USE_MOTORS_DEBUG) && (USE_MOTORS_DEBUG > 0)
    #define MOTORS_DEBUG(...)        printf(__VA_ARGS__)
#else
    #define MOTORS_DEBUG(...)
#endif

#define MOTOR_COUNT 2
//todo: check max speed
#define MOTOR_MAX_SPEED_RPM 2000
#define FULL_STEP_SPEED_RPM 100

#define MOTOR_MIN_ANGLE_DEG 0
#define MOTOR_MAX_ANGLE_DEG 90

#define MAX_SEEDRATE 300
#define MOTOR_CONTROL_LOOP_INTERVAL_IN_MS 100

#define STEPS_PER_REVOLUTION 200
#define DEG_TO_STEPS(deg) (((deg / 360.0) * (float)STEPS_PER_REVOLUTION)) * 16
#define RPM_TO_STEP_P_S(rpm) ((rpm * STEPS_PER_REVOLUTION) / 60.0)

typedef enum MotorState {
  Idle,
  ManualMode
} MotorState_t;

typedef struct Motor {
  bool running;
  uint32_t lastMotorControlTimeStamp;
  int32_t position;

  MotorState_t state;
  MotorState_t prevState;
} Motor_t;

Motor_t motors[MOTOR_COUNT];
void motors_Init();

#endif /* MOTORS_H_ */
