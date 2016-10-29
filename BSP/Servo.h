#ifndef _SERVO_H_
#define _SERVO_H_

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <ucos_ii.h>
#include "stm32f10x.h"
#include "RobotArmCtrlBoard.h"

#define ROBOTIC_ARM_NUM 10
#define MOVE_SLICE 8
#define STEP_GAP 20

void Servo_Run(void);

#endif