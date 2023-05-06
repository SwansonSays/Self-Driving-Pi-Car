#include <pthread.h>
#include <stdbool.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>  /* usleep() */
#include <stdlib.h>  /* exit() */
#include "MotorDriver.h"

#include <pigpio.h>

#include "sensor.h"


#define PIN_LINESENSOR_L    13
#define PIN_LINESENSOR_C    6
#define PIN_LINESENSOR_R    5

#define PIN_OBSTSENS_FRONTL 0
#define PIN_OBSTSENS_FRONTC 0
#define PIN_OBSTSENS_FRONTR 0
#define PIN_OBSTSENS_SIDEL  0
#define PIN_OBSTSENS_SIDER  0

#define NUM_LINE_SENSORS    3
#define NUM_OBST_SENSORS    5
#define NUM_MOTORS          2

#define MOTOR_LEFT  MOTORB
#define MOTOR_RIGHT MOTORA


static volatile bool terminate = false;


void handle_interrupt(int signal)
{
    //printf("\n\nInterrupt signal received. Terminating.\n");
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

    signal(SIGINT, handle_interrupt);

    int speed_left = 100;
    int speed_right = 100;

    /* GPIO pins for the line sensors */
    uint8_t line_sensor_pins[] = {
        (uint8_t)PIN_LINESENSOR_L,
        (uint8_t)PIN_LINESENSOR_C,
        (uint8_t)PIN_LINESENSOR_R
    };

    /* GPIO pins for the obstacle sensors */
    uint8_t obst_sensor_pins[] = {
        (uint8_t)PIN_OBSTSENS_FRONTL,
        (uint8_t)PIN_OBSTSENS_FRONTC,
        (uint8_t)PIN_OBSTSENS_FRONTR,
        (uint8_t)PIN_OBSTSENS_SIDEL,
        (uint8_t)PIN_OBSTSENS_SIDER
    };

    /* GPIO pins for the motor speed counter */
    uint8_t counter_pins[] = {
        (uint8_t)SPI0_CE0,
        (uint8_t)SPI0_CE1    
    };

    volatile uint8_t line_sensor_vals[NUM_LINE_SENSORS] = { 0 };
    SensorArgs* line_sensor_args[NUM_LINE_SENSORS];
    pthread_t line_sensor_threads[NUM_LINE_SENSORS];
    
    volatile uint8_t obst_sensor_vals[NUM_OBST_SENSORS] = { 0 };
    SensorArgs* obst_sensor_args[NUM_OBST_SENSORS];
    pthread_t obst_sensor_threads[NUM_OBST_SENSORS];

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

    /* Create thread routines for the obstacle sensors */
    for (size_t i = 0; i < NUM_OBST_SENSORS; i++) 
    {
        obst_sensor_args[i] = malloc(sizeof(SensorArgs));
        obst_sensor_args[i]->p_sensor_val = &obst_sensor_vals[i];
        obst_sensor_args[i]->gpio_pin = obst_sensor_pins[i];
        obst_sensor_args[i]->p_terminate = &terminate;
        /* Keep track of the pointer so it can be freed later */
        rc = pthread_create(&obst_sensor_threads[i], NULL, read_sensor, (void*)obst_sensor_args[i]);
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
    Motor_Run(MOTOR_LEFT, FORWARD, speed_left);
    Motor_Run(MOTOR_RIGHT, BACKWARD, speed_right);
    
    while (!terminate)
    {
        printf("%u, %u, %u\n", line_sensor_vals[0], line_sensor_vals[1], line_sensor_vals[2]);

        /* (1, 1, 1) All three sensors are on */
        if (line_sensor_vals[0] == HIGH && line_sensor_vals[1] == HIGH && line_sensor_vals[2] == HIGH)
        {
            printf("All three sensors are active\n");
        }
        /* (1, 1, 0) LEFT and CENTER */
        else if (line_sensor_vals[0] == HIGH && line_sensor_vals[1] == HIGH)
        {
        }
        /* (0, 1, 1) CENTER and RIGHT */
        else if (line_sensor_vals[1] == HIGH && line_sensor_vals[2] == HIGH)
        {
        }
        /* (1, 0, 1) LEFT and RIGHT */
        else if (line_sensor_vals[0] == HIGH && line_sensor_vals[2] == HIGH)
        {
            //printf("Left and Right\n");
        }
        /* (1, 0, 0) LEFT only */
        else if (line_sensor_vals[0] == HIGH)
        {
        }
        /* (0, 1, 0) CENTER only */
        else if (line_sensor_vals[1] == HIGH)
        {
            /* Motor speed should be equal. Set both to 100% */
        }
        /* (0, 0, 1) RIGHT only */
        else if (line_sensor_vals[2] == HIGH)
        {
        }
        /* (0, 0, 0) no sensors active */
        else 
        {
            printf("No sensors are active\n");
            Motor_Decrease_Speed(MOTOR_LEFT, speed_left, 0, 25);
            Motor_Decrease_Speed(MOTOR_RIGHT, speed_right, 0, 25);
            speed_left = speed_right = 0;
        }
        usleep(100);
    }
    /* Clean up the line sensor thread routines and memory */
    for (size_t i = 0; i < NUM_LINE_SENSORS; i++)
    {
        rc = pthread_join(line_sensor_threads[i], NULL);
        free(line_sensor_args[i]);
        line_sensor_args[i] = NULL;
    }
    /* Clean up the obstacle sensor thread routines and memory */
    for (size_t i = 0; i < NUM_OBST_SENSORS; i++) 
    {
        rc = pthread_join(obst_sensor_threads[i], NULL);
        free(obst_sensor_args[i]);
        obst_sensor_args[i] = NULL;
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
