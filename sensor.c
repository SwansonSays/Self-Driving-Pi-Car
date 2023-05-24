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
* File:         main.c
*
* Description:
*   Definitions for sensor reading functions. 
******************************************************************************/


#include "sensor.h"
#include <pigpio.h>
#include <unistd.h>     /* usleep() */
#include <stdio.h>


void read_sensor(SensorArgs* args)
{
    while(!*(args->p_terminate))
    {
        *(args->p_sensor_val) = gpioRead(args->gpio_pin);
        usleep(50);
    }
}

void read_counter(CounterArgs* args)
{
    int count = 0;
    double rpms = 0.0;
    
    while(!*(args->p_terminate))
    {    
        count = 0;
        rpms = 0;
        clearLS7336RCounter(args->chip_enable);

        usleep(10000);

        count = readLS7336RCounter(args->chip_enable);
        rpms = (100 * count) / (4 * PULSES_PER_REV);
        
        *(args->speed_val) = rpms * (2 * PI * WHEEL_RADIUS);
    }
}

