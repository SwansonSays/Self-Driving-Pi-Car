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

/**
 * Motor rotation.
 *
 * @param motor: Motor A and Motor B.
 * @param dir: forward and backward.
 * @param speed: Rotation speed.  //(0~100)
 *
 * Example:
 * @code
 * Motor_Run(MOTORA, FORWARD, 50);
 * Motor_Run(MOTORB, BACKWARD, 100);
 */

void Motor_Change_Direc(DIR dir){
    PCA9685_SetPwmDutyCycle(PCA_CHANNEL_0, 0);
    sleep(1);
    if(dir == FORWARD) {
        DEBUG("forward...\r\n");
        PCA9685_SetLevel(PCA_CHANNEL_1, 0);
        PCA9685_SetLevel(PCA_CHANNEL_2, 1);
    } else {
        DEBUG("backward...\r\n");
        PCA9685_SetLevel(PCA_CHANNEL_1, 1);
        PCA9685_SetLevel(PCA_CHANNEL_2, 0);
    }
}


//accelerate from the current speed to accel. rate is in seconds and increments speed by 5/sec
void Motor_Increase_Speed(UBYTE motor,UWORD current, UWORD accel, int rate){

	for(int i = current+1;i<=accel;i++){
		//DEBUG("Motor Speed = %d\r\n", i);
		if(current == MOTORA){
			PCA9685_SetPwmDutyCycle(PCA_CHANNEL_0, i);
		}else if(motor == MOTORB){
			PCA9685_SetPwmDutyCycle(PCA_CHANNEL_1, i);
		}
		if((i!=0)&&i%rate==0){
			sleep(1);
		}
	}

}


void Motor_Decrease_Speed(UBYTE motor,UWORD current, UWORD slow, int rate){

        for(int i = current-1;i>=slow;i--){
		//DEBUG("Motor Speed = %d\r\n", i);
                if(current == MOTORA){
                        PCA9685_SetPwmDutyCycle(PCA_CHANNEL_0, i);
                }else if(motor == MOTORB){
                        PCA9685_SetPwmDutyCycle(PCA_CHANNEL_1, i);
                }
		if((i!=0)&&(i%rate==0)){
                	sleep(1);
		}
        }

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
