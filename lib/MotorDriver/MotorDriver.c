/*****************************************************************************
* | File        :   MotorDriver.c
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
#include "MotorDriver.h"
#include "Debug.h"

/**
 * Motor rotation.
 *
 * Example:
 * Motor_Init();
 */
void Motor_Init(void)
{
    PCA9685_Init(0x40);
    PCA9685_SetPWMFreq(100);
}


void Motor_Set_Direction(UBYTE motor, DIR direction, UWORD speed)
{
    Motor_Stop(motor);
    Motor_Run(motor, direction, speed);
}


/**
 * Accelerate from the current power level to desired.
 * Duty cycle is updated in increments of `rate`.
 * Maximum desired power level is 100.
 * 
 * Returns the updated power level of the given motor.
 */
UWORD Motor_Increase_Speed(UBYTE motor, UWORD current, UWORD desired, int rate)
{
    if (desired > 100) { desired = 100; }

    while (current < desired)
    {
        current += rate;

        /* Maximum allowed duty cycle is 100% */
        if (current > 100) { 
            current = 100; 
        }
        /* Avoid overshooting the target value */
        if (current > desired) {
            current = desired;
        }
        /* Update the duty cycle for the given motor */
        if (motor == MOTORA) {
            PCA9685_SetPwmDutyCycle(PWMA, current);
        } 
        else if (motor == MOTORB) {
            PCA9685_SetPwmDutyCycle(PWMB, current);
        }
    }
    return current;
}


/**
 * Decelerate from the current power level to desired.
 * Duty cycle is updated in increments of `rate`.
 * Minimum desired power level is 0.
 * 
 * Returns the updated power level of the given motor.
 */
UWORD Motor_Decrease_Speed(UBYTE motor, UWORD current, UWORD desired, int rate)
{
    if (desired < 0) { desired = 0; }

    while (current > desired)
    {
        current -= rate;

        /* Prevent setting a negative duty cycle */
        if (current < 0) { 
            current = 0; 
        }
        /* Prevent overshooting the desired value */
        if (current < desired) { 
            current = desired; 
        }
        /* Update the duty cycle for the given motor */
        if (motor == MOTORA) {
            PCA9685_SetPwmDutyCycle(PWMA, current);
        } 
        else if (motor == MOTORB) {
            PCA9685_SetPwmDutyCycle(PWMB, current);
        }
    }
    return current;
}


void Motor_Run(UBYTE motor, DIR dir, UWORD speed)
{
    if(speed > 100)
        speed = 100;

    if(motor == MOTORA) {
        DEBUG("Motor A Speed = %d\r\n", speed);
        PCA9685_SetPwmDutyCycle(PWMA, speed);
        if(dir == FORWARD) {
            DEBUG("forward...\r\n");
            PCA9685_SetLevel(AIN1, 0);
            PCA9685_SetLevel(AIN2, 1);
        } else {
            DEBUG("backward...\r\n");
            PCA9685_SetLevel(AIN1, 1);
            PCA9685_SetLevel(AIN2, 0);
        }
    } else {
        DEBUG("Motor B Speed = %d\r\n", speed);
        PCA9685_SetPwmDutyCycle(PWMB, speed);
        if(dir == FORWARD) {
            DEBUG("forward...\r\n");
            PCA9685_SetLevel(BIN1, 0);
            PCA9685_SetLevel(BIN2, 1);
        } else {
            DEBUG("backward...\r\n");
            PCA9685_SetLevel(BIN1, 1);
            PCA9685_SetLevel(BIN2, 0);
        }
    }
}

/**
 * Motor stop rotation.
 *
 * @param motor: Motor A and Motor B.
 *
 * Example:
 * @code
 * Motor_Stop(MOTORA);
 */
void Motor_Stop(UBYTE motor)
{
    if(motor == MOTORA) {
        PCA9685_SetPwmDutyCycle(PWMA, 0);
    } else {
        PCA9685_SetPwmDutyCycle(PWMB, 0);
    }
}
