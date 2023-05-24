 /******************************************************************************
* Class:        CSC-615-01 Spring 2023
*
* Names:        Zachary Colbert
*               Sajan Gurung
*               Robert Swanson
*               Tyler Wartzok
*
* Github ID:    ttwartzok
* Project:      Final Project - Self Driving Car 
*
* File:         sensor.c
*
* Description:
*   Definitions for sensor reading functions. 
******************************************************************************/


#include "sensor.h"
#include <pigpio.h>
#include <unistd.h>     /* usleep() */
#include <stdio.h>


/**
 * Thread routine to monitor a line sensor.
 */
void read_sensor(SensorArgs* args)
{
    while(!*(args->p_terminate))
    {
        *(args->p_sensor_val) = gpioRead(args->gpio_pin);
        usleep(50);
    }
}

