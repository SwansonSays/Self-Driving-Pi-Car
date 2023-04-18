#include "linesensor.h"
#include <pigpio.h>
#include <stdio.h>


void read_sensor(SensorArgs* args)
{
    while (1)
    {
        args->sensors[args->index] = gpioRead(args->gpio_pin);
        usleep(50);
    }
}
