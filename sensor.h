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
#include "lib/7366r/7366rDriver.h"


#define HIGH    1
#define LOW     0


typedef struct {
  uint8_t value;
  uint8_t last_value;
  uint8_t confidence;
} Sensor;

typedef struct {
    uint8_t* p_sensor_val;
    uint8_t gpio_pin;
    bool* p_terminate;
} SensorArgs;

typedef struct {
  double* speed_val;
  uint8_t chip_enable;
  bool* p_terminate;
} CounterArgs;

void read_sensor(SensorArgs* args);
void read_counter(CounterArgs* args);


#endif  /* _SENSOR_H */
