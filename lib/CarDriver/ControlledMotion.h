#ifndef __CAR_H__
#define __CAR_H__

#include <stdio.h>      //printf()
#include <stdlib.h>     //exit()
#include <signal.h>

#include "DEV_Config.h"
#include <time.h>
#include <pigpio.h>
#include "DEV_Config.h"
#include "MotorDriver.h"
#include "7366rDriver.h"
#include"../definitons.h"
#include <unistd.h>
//#define SPI0_CE0        GPIO08          //Physical Pin 24
//#define SPI0_CE1        GPIO07   


//TODO: CREATE SOME TYPE OF CONTROLBLOCK FOR THE CAR

//if MOTORA then use SPI0_CE0 otherwise SPI_CE1 to check count
double revsPerSec(UWORD motor);


//checks if both motors are producing equal rpms, if not loop until equal, loop has internal 3 scond delay to confirm rpm match is
// not just momentary
int syncRPMS(double &powerA, double &powerB);

#endif
