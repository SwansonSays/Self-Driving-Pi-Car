#include <pthread.h>
#include <stdbool.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>  /* usleep() */
#include <stdlib.h>  /* exit() */
#include "MotorDriver.h"

#include <pigpio.h>

#include "sensor.h"
#include "movement.h"


#define PIN_LINESENSOR_L    5
#define PIN_LINESENSOR_C    6
#define PIN_LINESENSOR_R    13

#define NUM_LINE_SENSORS    3
#define NUM_OBST_SENSORS    5
#define NUM_MOTORS          2


static volatile bool terminate = false;

void handle_interrupt(int signal)
{
    terminate = true;
}


int main(int argc, char* argv[])
{
    /* Initialize motor driver */
    if(DEV_ModuleInit()) 
    {
        exit(1);
    }

    /* Initialize pigpio library */
    if (gpioInitialise() < 0) 
    {
        fprintf(stderr, "Failed to initialize pigpio\n");
        exit(1);
    }

    if (initLS7336RChip(SPI0_CE0) || initLS7336RChip(SPI0_CE1))
    {
        printf("Error initializing the LS7336R chip.\n");
        DEV_ModuleExit();
        gpioTerminate();
        exit(1);
    }

    Motor_Init();

    ProgramState state;
    state.last_dir = STRAIGHT;
    state.last_req = STRAIGHT;
    state.speed_left = 100;
    state.speed_right = 100;
    state.confidence = 0;

    signal(SIGINT, handle_interrupt);

    /* GPIO pins for the line sensors */
    uint8_t line_sensor_pins[] = {
        (uint8_t)PIN_LINESENSOR_L,
        (uint8_t)PIN_LINESENSOR_C,
        (uint8_t)PIN_LINESENSOR_R
    };

    /* GPIO pins for the motor speed counter */
    uint8_t counter_pins[] = {
        (uint8_t)SPI0_CE0,
        (uint8_t)SPI0_CE1    
    };

    volatile uint8_t line_sensor_vals[NUM_LINE_SENSORS] = { 0 };
    SensorArgs* line_sensor_args[NUM_LINE_SENSORS];
    pthread_t line_sensor_threads[NUM_LINE_SENSORS];
    
    volatile double hall_sensor_vals[NUM_MOTORS] = { 0 };    
    CounterArgs* hall_sensor_args[NUM_MOTORS];
    pthread_t hall_sensor_threads[NUM_MOTORS];

    int rc;
    /* Create thread routines for the line sensors */
    for (size_t i = 0; i < NUM_LINE_SENSORS; i++)
    {
        line_sensor_args[i] = malloc(sizeof(SensorArgs));
        line_sensor_args[i]->p_sensor_val = &line_sensor_vals[i];
        line_sensor_args[i]->gpio_pin = line_sensor_pins[i];
        line_sensor_args[i]->p_terminate = &terminate;
        /* Keep track of the pointer so it can be freed later */
        rc = pthread_create(&line_sensor_threads[i], NULL, read_sensor, (void*)line_sensor_args[i]);
    }

    /* Create thread routines for the motors */
    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        hall_sensor_args[i] = malloc(sizeof(CounterArgs));
        hall_sensor_args[i]->speed_val = &hall_sensor_vals[i];
        hall_sensor_args[i]->chip_enable = counter_pins[i];
        hall_sensor_args[i]->p_terminate = &terminate;
        /* Keep track of the pointer so it can be freed later */
        rc = pthread_create(&hall_sensor_threads[i], NULL, read_counter, (void*)hall_sensor_args[i]);
    }

    /* Directions must be alternated because the motors are mounted
     * in opposite orientations. Both motors will turn forward relative 
     * to the car. */
    Motor_Run(MOTOR_LEFT, FORWARD, state.speed_left);
    Motor_Run(MOTOR_RIGHT, BACKWARD, state.speed_right);
    

    while (!terminate)
    {
        printf("%u, %u, %u (%d)\n", line_sensor_vals[0], line_sensor_vals[1], line_sensor_vals[2], state.confidence);
        follow_line(line_sensor_vals, &state);
        usleep(1000);
    }
    /* Clean up the line sensor thread routines and memory */
    for (size_t i = 0; i < NUM_LINE_SENSORS; i++)
    {
        rc = pthread_join(line_sensor_threads[i], NULL);
        free(line_sensor_args[i]);
        line_sensor_args[i] = NULL;
    }
    /* Clean up the motor thread routines and memory */
    for (size_t i = 0; i < 2; i++)
    {
        rc = pthread_join(hall_sensor_threads[i], NULL);
        free(hall_sensor_args[i]);
        hall_sensor_args[i] = NULL;
    } 
    Motor_Stop(MOTORA);
    Motor_Stop(MOTORB);

    DEV_ModuleExit();
    gpioTerminate();
    pthread_exit(NULL);

    return 0;
}
