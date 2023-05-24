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
* File:         sensor.h
*
* Description:
*   Declarations for sensor reading functions and related structures. 
******************************************************************************/

#ifndef _SENSOR_H
#define _SENSOR_H


#include <stdbool.h>
#include <stdint.h>  /* uint8_t */
#include <pthread.h>


#define HIGH    1
#define LOW     0


typedef struct {
    uint8_t* p_sensor_val;
    uint8_t gpio_pin;
    bool* p_terminate;
} SensorArgs;


void read_sensor(SensorArgs* args);


#endif  /* _SENSOR_H */
