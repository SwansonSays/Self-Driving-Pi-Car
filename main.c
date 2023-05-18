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
#include "movement.h"


#define PIN_LINESENSOR_FRONT_L    5
#define PIN_LINESENSOR_FRONT_C    6
#define PIN_LINESENSOR_FRONT_R 	  13

#define PIN_LINESENSOR_REAR_L     23
#define PIN_LINESENSOR_REAR_R	  24

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
}

struct Params
{
    /* Pointer to shared struct */
    struct Lidar_data* shared;
    /* Params to set the viewing angle */
    float right_theta;
    float left_theta;
    float max_distance;
    /* Values of the most recent valid sensor reading */
    float theta;
    float distance;
};

struct Lidar_data {
    float theta;
    float distance;
    int quality;
}Lidar_data;

void read_lidar(void* args) {
    struct Lidar_data* temp_data = malloc(sizeof(struct Lidar_data));

    while (1) {
        /* 
        *   Copy lidar scan to temparay struct because lidar scans faster then we can 
        *   proccess the data This avoids our data being overwritten while proccessing 
        */
        memcpy(temp_data, (struct Params)args->shared, sizeof(struct Lidar_data));

        /* If quality of the scan is good and the distance is greater then 0 but less then max */
        if (temp_data->quality > 10 && temp_data->distance < (struct Params)args->max_distance && temp_data->distance > 0) {
            /* If scan is inside our viewing range */
            if (temp_data->theta > (struct Params*)args->left_theta || temp_data->theta < args->right_theta) {
                printf("THETA [%f] | DISTANCE [%f] | QUALITY [%d]\n", temp_data->theta, temp_data->distance, temp_data->quality);
                args.theta = temp_data->theta;
                args.distance = temp_data->distance;
            }
        }
    }
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

    /* Initilize shared memory */
    const char* name = "SHARED_MEMORY";
    int shm_fd = shm_open(name, O_RDONLY, 0666);
    struct Lidar_data* data = mmap(NULL, sizeof(struct Lidar_data), PROT_READ, MAP_SHARED, shm_fd, 0);

    struct Params params;

    params.shared = Lidar_data;
    params.left_theta = 270.0f;
    params.right_theta = 90.0f;
    params.max_distance = 400.0f;
    params.theta = 0.0f;
    params.distance = 0.0f;


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

    pthread_t lidar_thread;

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

    /* cerate thread routines for the lidar */
    pthread_create(&lidar_thread, NULL, read_lidar, (void*)&params);

    /* Directions must be alternated because the motors are mounted
     * in opposite orientations. Both motors will turn forward relative 
     * to the car. */
    Motor_Run(MOTOR_LEFT, FORWARD, state.speed_left);
    Motor_Run(MOTOR_RIGHT, BACKWARD, state.speed_right);

    while (!terminate)
    {
        printf("%u, %u, %u / (%d, %d) Confidence: (%d / %d)\n", 
            line_sensor_vals[0], line_sensor_vals[1], line_sensor_vals[2], 
            line_sensor_vals[3], line_sensor_vals[4], 
            state.inner_confidence, state.outer_confidence);

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
    /* Clean up the lidar thread routines and shared memory */
    munmap(NULL, sizeof(*data));
    shm_unlink(name);
    close(shm_fd);

    Motor_Stop(MOTORA);
    Motor_Stop(MOTORB);

    DEV_ModuleExit();
    gpioTerminate();
    pthread_exit(NULL);

    return 0;
}
