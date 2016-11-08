#ifndef _SERVO_H_
#define _SERVO_H_

/* Includes */
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <ucos_ii.h>
#include "stm32f10x.h"
#include "RobotArmCtrlBoard.h"
#include "s_curve.h"

/* Defines */
#define ROBOTIC_ARM_NUM 10
#define SERVOS_USED 0x017F

/* Global Variables */
extern char* servo_name[ROBOTIC_ARM_NUM];
extern OS_EVENT *SemApply[ROBOTIC_ARM_NUM];
extern char* sem_name[ROBOTIC_ARM_NUM];

extern float cur_angle[ROBOTIC_ARM_NUM];
extern float target_angle[ROBOTIC_ARM_NUM];
extern const float ang_min[ROBOTIC_ARM_NUM];
extern const float ang_max[ROBOTIC_ARM_NUM];
extern uint16_t cur_pwm[ROBOTIC_ARM_NUM];
extern uint16_t target_pwm[ROBOTIC_ARM_NUM];
extern const uint16_t pwm_min[ROBOTIC_ARM_NUM];
extern const uint16_t pwm_max[ROBOTIC_ARM_NUM];
extern const uint16_t pwm_resolution[ROBOTIC_ARM_NUM];

/* Function Prototypes */
int min(int a, int b);
int max(int a, int b);
uint16_t angle2pwm(int i, float _ang);
float pwm2ang(int i, uint16_t _pwm);
extern void move(int i);
void advance_to_next(int i);
void pos_update(int index);
void Servo_InitMove(void);
void Servo_Run(void);
void optimized_move(int index);
void unguarded_move_one(int index, uint16_t _pwm_pos);

#endif