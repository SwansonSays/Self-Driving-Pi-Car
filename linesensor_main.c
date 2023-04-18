#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>  /* usleep() */
#include <pigpio.h>

#include "linesensor.h"

#define PIN_LINESENSOR_L    17
#define PIN_LINESENSOR_C    27
#define PIN_LINESENSOR_R    22

#define IDX_LINESENSOR_L    0
#define IDX_LINESENSOR_C    1
#define IDX_LINESENSOR_R    2

#define VAL_LINESENSOR_L    0x01
#define VAL_LINESENSOR_C    0x02
#define VAL_LINESENSOR_R    0x04


static volatile int terminate = 0;

void handle_interrupt(int signal)
{
    printf("\n\nInterrupt signal received. Terminating.\n");
    terminate = 1;
}


int main(int argc, char* argv[])
{
    gpioInitialise();

    signal(SIGINT, handle_interrupt);

    uint8_t line_sensor_vals[3] = { 0 };

    SensorArgs args_left;
    args_left.sensors=line_sensor_vals;
    args_left.gpio_pin=(uint8_t)PIN_LINESENSOR_L;
    args_left.index=IDX_LINESENSOR_L;

    SensorArgs args_center;
    args_center.sensors=line_sensor_vals;
    args_center.gpio_pin=(uint8_t)PIN_LINESENSOR_C;
    args_center.index=IDX_LINESENSOR_C;

    SensorArgs args_right;
    args_right.sensors=line_sensor_vals;
    args_right.gpio_pin=(uint8_t)PIN_LINESENSOR_R;
    args_right.index=IDX_LINESENSOR_R;

    pthread_t threads[3];

    pthread_create(&threads[0], NULL, read_sensor, (void*)&args_left);
    pthread_create(&threads[1], NULL, read_sensor, (void*)&args_center);
    pthread_create(&threads[1], NULL, read_sensor, (void*)&args_right);


    while (!terminate)
    {
        printf("%u, %u, %u\n\n", line_sensor_vals[0], line_sensor_vals[1], line_sensor_vals[2]);
        usleep(100);

        /* (1, 1, 1) All three sensors are on */
        if (line_sensor_vals[0] == LOW && line_sensor_vals[1] == LOW && line_sensor_vals[2] == LOW)
        {
            printf("All three sensors are active\n");
        }
        /* (1, 1, 0) LEFT and CENTER */
        else if (line_sensor_vals[0] == LOW && line_sensor_vals[1] == LOW)
        {
            printf("Left and Center\n");
        }
        /* (0, 1, 1) CENTER and RIGHT */
        else if (line_sensor_vals[1] == LOW && line_sensor_vals[2] == LOW)
        {
            printf("Center and Right\n");
        }
        /* (1, 0, 1) LEFT and RIGHT */
        else if (line_sensor_vals[0] == LOW && line_sensor_vals[2] == LOW)
        {
            printf("Left and Right\n");
        }
        /* (1, 0, 0) LEFT only */
        else if (line_sensor_vals[0] == LOW)
        {
            printf("Left only\n");
        }
        /* (0, 1, 0) CENTER only */
        else if (line_sensor_vals[1] == LOW)
        {
            printf("Center only\n");
        }
        /* (0, 0, 1) RIGHT only */
        else if (line_sensor_vals[2] == LOW)
        {
            printf("Right only\n");
        }
        /* (0, 0, 0) no sensors active */
        else 
        {
            printf("No sensors are active\n");
        }
    }

    pthread_join(&threads[0], NULL);
    pthread_join(&threads[1], NULL);
    pthread_join(&threads[2], NULL);

    gpioTerminate();

    pthread_exit(NULL);
    return 0;
}
