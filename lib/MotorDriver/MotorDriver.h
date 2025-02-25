/*****************************************************************************
* | File        :   MotorDriver.h
* | Author      :   Waveshare team
* | Function    :   Drive TB6612FNG
* | Info        :
*                TB6612FNG is a driver IC for DC motor with output transistor in
*                LD MOS structure with low ON-resistor. Two input signals, IN1
*                and IN2, can choose one of four modes such as CW, CCW, short
*                brake, and stop mode.
*----------------
* |	This version:   V1.0
* | Date        :   2018-09-04
* | Info        :   Basic version
*
******************************************************************************/
#ifndef __TB6612FNG_
#define __TB6612FNG_

#include "DEV_Config.h"
#include "PCA9685.h"

//GPIO config
#define PWMA        PCA_CHANNEL_0
#define AIN1        PCA_CHANNEL_1
#define AIN2        PCA_CHANNEL_2
#define PWMB        PCA_CHANNEL_5
#define BIN1        PCA_CHANNEL_3
#define BIN2        PCA_CHANNEL_4

#define MOTORA       0
#define MOTORB       1

#define MOTOR_LEFT  MOTORA
#define MOTOR_RIGHT MOTORB

/* Account for opposite mounting orientations of motors */
#define MOTOR_LEFT_FORWARD      FORWARD
#define MOTOR_LEFT_BACKWARD     BACKWARD

#define MOTOR_RIGHT_FORWARD     BACKWARD
#define MOTOR_RIGHT_BACKWARD    FORWARD

typedef enum {
    FORWARD  = 1,
    BACKWARD  ,
    STRAIGHT,
    LEFT,
    RIGHT
} DIR;

void Motor_Init(void);
void Motor_Run(UBYTE motor, DIR dir, UWORD speed);

void Motor_Stop(UBYTE motor);

void Motor_Change_Direc(DIR dir);
void Motor_Set_Direction(UBYTE motor, DIR direction, UWORD speed);

UWORD Motor_Increase_Speed(UBYTE motor,UWORD current,UWORD accel,int rate);
UWORD Motor_Decrease_Speed(UBYTE motor,UWORD current,UWORD slow,int rate);

#endif
