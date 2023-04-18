#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#include <stdint.h>  /* uint8_t */
#include <pthread.h>


#define HIGH 1
#define LOW  0


typedef struct {
    uint8_t* sensors;
    uint8_t gpio_pin;
    uint8_t index;
} SensorArgs;


void read_sensor(SensorArgs* args);


#endif  /* _LINESENSOR_H */
