#include "sensor.h"
#include <pigpio.h>
#include <stdio.h>


void read_sensor(SensorArgs* args)
{
    while (!*(args->p_terminate))
    {
        *(args->p_sensor_val) = gpioRead(args->gpio_pin);
        usleep(50);
    }
}

