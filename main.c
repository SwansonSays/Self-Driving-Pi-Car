#include <pthread.h>
#include <stdbool.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>  /* usleep() */
#include <stdlib.h>  /* exit() */

#include <pigpio.h>

#include "sensor.h"


#define PIN_LINESENSOR_L    17
#define PIN_LINESENSOR_C    27
#define PIN_LINESENSOR_R    22

#define PIN_OBSTSENS_FRONTL 6
#define PIN_OBSTSENS_FRONTC 5
#define PIN_OBSTSENS_FRONTR 13
#define PIN_OBSTSENS_SIDEL 16
#define PIN_OBSTSENS_SIDER 16


#define NUM_LINE_SENSORS    3
#define NUM_OBST_SENSORS    5


static volatile bool terminate = false;


void handle_interrupt(int signal)
{
    printf("\n\nInterrupt signal received. Terminating.\n");
    terminate = true;
}


int main(int argc, char* argv[])
{
    if (gpioInitialise() < 0) 
    {
        fprintf(stderr, "Failed to initialize pigpio\n");
        exit(1);
    }

    signal(SIGINT, handle_interrupt);

    uint8_t line_sensor_pins[] = {
        (uint8_t)PIN_LINESENSOR_L,
        (uint8_t)PIN_LINESENSOR_C,
        (uint8_t)PIN_LINESENSOR_R
    };

    uint8_t line_sensor_vals[NUM_LINE_SENSORS] = { 0 };
    SensorArgs* line_sensor_args[NUM_LINE_SENSORS];
    pthread_t line_sensor_threads[NUM_LINE_SENSORS];

    uint8_t obst_sensor_pins[] = {
        (uint8_t)PIN_OBSTSENS_FRONTL,
        (uint8_t)PIN_OBSTSENS_FRONTC,
        (uint8_t)PIN_OBSTSENS_FRONTR,
        (uint8_t)PIN_OBSTSENS_SIDEL,
        (uint8_t)PIN_OBSTSENS_SIDER
    };

    uint8_t obst_sensor_vals[NUM_OBST_SENSORS] = { 0 };
    SensorArgs* obst_sensor_args[NUM_OBST_SENSORS];
    pthread_t obst_sensor_threads[NUM_LINE_SENSORS];

    for (size_t i = 0; i < NUM_LINE_SENSORS; ++i)
    {
        SensorArgs* args = malloc(sizeof(SensorArgs));
        args->p_sensor_val = &line_sensor_vals[i];
        args->gpio_pin = line_sensor_pins[i];
        args->p_terminate = &terminate;
        /* Keep track of the pointer so it can be freed later */
        line_sensor_args[i] = args;
        pthread_create(&line_sensor_threads[i], NULL, read_sensor, (void*)args);
    }

    for (size_t i = 0; i < NUM_OBST_SENSORS; i++) {
        SensorArgs* args = malloc(sizeof(SensorArgs));
        args->p_sensor_val = &obst_sensor_vals[i];
        args->gpio_pin = obst_sensor_pins[i];
        args->p_terminate = &terminate;

        obst_sensor_args[i] = args;
        pthread_create(&obst_sensor_threads[i], NULL, read_sensor, (void*)args);
    }

    while (!terminate)
    {
        printf("%u, %u, %u\n\n", line_sensor_vals[0], line_sensor_vals[1], line_sensor_vals[2]);

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
        usleep(100);
    }

    /* Clean up the line sensor thread routines and memory */
    for (size_t i = 0; i < NUM_LINE_SENSORS; ++i)
    {
        pthread_join(&line_sensor_threads[i], NULL);
        free(line_sensor_args[i]);
        line_sensor_args[i] = NULL;
    }

    for (size_t i = 0; i < NUM_OBST_SENSORS; ++i) {
        pthread_join(&obst_sensor_threads[i], NULL);
        free(obst_sensor_args[i]);
        obst_sensor_args[i] = NULL;
    }

    gpioTerminate();
    pthread_exit(NULL);
    return 0;
}
