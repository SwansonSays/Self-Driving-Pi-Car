#include <pthread.h>
#include <stdbool.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>     /* usleep() and close() */
#include <stdlib.h>     /* exit() */
#include <sys/mman.h>   /* shm_open() and mmap() */
#include <sys/stat.h>   /* For mode constants */
#include <fcntl.h>      /* For O_* constants */
#include <string.h>     /* memcpy() */
#include "MotorDriver.h"

#include <pigpio.h>

#include "sensor.h"
#include "sonar.h"
#include "movement.h"

#include <errno.h>


#define PIN_LINESENSOR_FRONT_L    5
#define PIN_LINESENSOR_FRONT_C    6
#define PIN_LINESENSOR_FRONT_R 	  13

#define PIN_LINESENSOR_REAR_L     23
#define PIN_LINESENSOR_REAR_R	  24

#define PIN_SONAR_FRONT_ECHO      12
#define PIN_SONAR_FRONT_TRIG      16

#define PIN_SONAR_LEFT_ECHO       20
#define PIN_SONAR_LEFT_TRIG       21

#define NUM_LINE_SENSORS    5
#define NUM_MOTORS          2


static volatile bool terminate = false;

void handle_interrupt(int signal)
{
    terminate = true;
}

void init_program_state(ProgramState* state)
{
    state->last_dir = STRAIGHT;
    state->last_req = STRAIGHT;
    state->speed_left = 100;
    state->speed_right = 100;
    state->inner_confidence = 0;
    state->outer_confidence = 0;
    state->mode = LINE;
    state->p_terminate = &terminate;
}

int main(int argc, char* argv[])
{
    /* Initialize motor driver */
    if(DEV_ModuleInit()) {
        exit(1);
    }
    /* Initialize pigpio library */
    if (gpioInitialise() < 0) 
    {
        fprintf(stderr, "Failed to initialize pigpio\n");
        exit(1);
    }
    if (gpioSetMode(PIN_LINESENSOR_FRONT_L, PI_INPUT)
        || gpioSetMode(PIN_LINESENSOR_FRONT_C, PI_INPUT)
        || gpioSetMode(PIN_LINESENSOR_FRONT_R, PI_INPUT)
        || gpioSetMode(PIN_LINESENSOR_REAR_L, PI_INPUT)
        || gpioSetMode(PIN_LINESENSOR_REAR_R, PI_INPUT)
        || gpioSetMode(PIN_SONAR_FRONT_ECHO, PI_INPUT)
        || gpioSetMode(PIN_SONAR_FRONT_TRIG, PI_OUTPUT)
        || gpioSetMode(PIN_SONAR_LEFT_ECHO, PI_INPUT)
        || gpioSetMode(PIN_SONAR_LEFT_TRIG, PI_OUTPUT))
    {
        fprintf(stderr, "Failed to set pin modes\n");
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
    init_program_state(&state);

    signal(SIGINT, handle_interrupt);

    /* GPIO pins for the line sensors */
    uint8_t line_sensor_pins[] = {
        (uint8_t)PIN_LINESENSOR_FRONT_L,
        (uint8_t)PIN_LINESENSOR_FRONT_C,
        (uint8_t)PIN_LINESENSOR_FRONT_R,
        (uint8_t)PIN_LINESENSOR_REAR_L,
        (uint8_t)PIN_LINESENSOR_REAR_R
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

    pthread_t sonar_thread_front;
    pthread_t sonar_thread_left;

    SonarArgs sonar_args_front;
    init_SonarArgs(&sonar_args_front, 
        (uint8_t)PIN_SONAR_FRONT_TRIG, 
        (uint8_t)PIN_SONAR_FRONT_ECHO);
    sonar_args_front.p_terminate = &terminate;

    SonarArgs sonar_args_left;
    init_SonarArgs(&sonar_args_left, 
        (uint8_t)PIN_SONAR_LEFT_TRIG, 
        (uint8_t)PIN_SONAR_LEFT_ECHO);
    sonar_args_left.p_terminate = &terminate;

    /* Create thread routines for the line sensors */
    for (size_t i = 0; i < NUM_LINE_SENSORS; i++)
    {
        line_sensor_args[i] = malloc(sizeof(SensorArgs));
        line_sensor_args[i]->p_sensor_val = &line_sensor_vals[i];
        line_sensor_args[i]->gpio_pin = line_sensor_pins[i];
        line_sensor_args[i]->p_terminate = &terminate;
        pthread_create(&line_sensor_threads[i], NULL, read_sensor, (void*)line_sensor_args[i]);
    }
    /* Create thread routines for the motors */
    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        hall_sensor_args[i] = malloc(sizeof(CounterArgs));
        hall_sensor_args[i]->speed_val = &hall_sensor_vals[i];
        hall_sensor_args[i]->chip_enable = counter_pins[i];
        hall_sensor_args[i]->p_terminate = &terminate;
        pthread_create(&hall_sensor_threads[i], NULL, read_counter, (void*)hall_sensor_args[i]);
    }
    /* Create thread routines for the sonar sensors */
    pthread_create(&sonar_thread_front, NULL, watch_sonar, (void*)&sonar_args_front);
    pthread_create(&sonar_thread_left, NULL, watch_sonar, (void*)&sonar_args_left);

    /* Directions must be alternated because the motors are mounted
     * in opposite orientations. Both motors will turn forward relative 
     * to the car. */
    Motor_Run(MOTOR_LEFT, MOTOR_LEFT_FORWARD, state.speed_left);
    Motor_Run(MOTOR_RIGHT, MOTOR_RIGHT_FORWARD, state.speed_right);

    float front_obstacle_range_cm = 10.0f;
    float left_obstacle_range_cm = 30.0f;
    while (!terminate)
    {
        //printf("FRONT: %5.2f (%d)\n", sonar_args_front.distance_cm, sonar_args_front.confidence);
        //printf("LEFT: %5.2f (%d)\n", sonar_args_left.distance_cm, sonar_args_left.confidence);

        if (object_present(&sonar_args_front, front_obstacle_range_cm)) {
            //printf("Object at front %6.2f (%d)\n", sonar_args_front.distance_cm, sonar_args_front.confidence);
            avoid_obstacle(&sonar_args_front, &sonar_args_left, &state, line_sensor_vals);
        }
        follow_line(line_sensor_vals, &state);
        usleep(1000);
    }
    pthread_join(sonar_thread_front, NULL);
    pthread_join(sonar_thread_left, NULL);

    /* Clean up the line sensor thread routines and memory */
    for (size_t i = 0; i < NUM_LINE_SENSORS; i++)
    {
        pthread_join(line_sensor_threads[i], NULL);
        free(line_sensor_args[i]);
        line_sensor_args[i] = NULL;
    }
    /* Clean up the motor thread routines and memory */
    for (size_t i = 0; i < 2; i++)
    {
        pthread_join(hall_sensor_threads[i], NULL);
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
