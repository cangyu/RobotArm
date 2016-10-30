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
#define SERVOS_USED 0x017F

extern char* servo_name[ROBOTIC_ARM_NUM];
extern float cur_angle[ROBOTIC_ARM_NUM];
extern float target_angle[ROBOTIC_ARM_NUM];
extern float ang_min[ROBOTIC_ARM_NUM];
extern float ang_max[ROBOTIC_ARM_NUM];
extern int cur_pwm[ROBOTIC_ARM_NUM];
extern int target_pwm[ROBOTIC_ARM_NUM];
extern int pwm_min[ROBOTIC_ARM_NUM];
extern int pwm_max[ROBOTIC_ARM_NUM];
extern int gap[ROBOTIC_ARM_NUM];
extern int pwm_resolution[ROBOTIC_ARM_NUM];
extern OS_EVENT *SemApply[ROBOTIC_ARM_NUM];
extern char* sem_name[ROBOTIC_ARM_NUM];

int min(int a, int b);
int max(int a, int b);
int angle2pwm(int i, float _ang);
float pwm2ang(int i, int _pwm);
void move(int i);
void advance_one(int i);
void pos_update(int index);
void Servo_InitMove(void);
void Servo_Run(void);

#endif