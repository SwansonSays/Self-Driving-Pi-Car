#include "sonar.h"

/**
 * Calculate a distance in meters based on elapsed time in nanoseconds.
 *
 * Since the signal traveled to the object and back, the initial time 
 * reflects twice the actual distance (for the entire round trip) so 
 * the measured distance will be half of that which sound could travel 
 * in the elapsed time.
 *
 * @param time_ns The elapsed time in nanoseconds.
 * @return The distance in meters.
 */
float distance_m(time_t time_ns)
{
    return 0.5 * VSOUND_M_NS * time_ns ;
}


void read_sonar(SonarArgs* args)
{
    struct timespec start_time;     /* Time when trigger signal was sent */
    struct timespec end_time;       /* Time when echo signal was received */
    time_t time_elapsed_ns = 0;     /* Time spent waiting for echo signal */

    struct timespec timeout_start;       
    struct timespec timeout_now;       
    time_t timeout_max = 1000000000;

    bool valid_reading = true;

    while(!*(args->p_terminate))
    {
        valid_reading = true;
        /* Send a signal for 10 microseconds */
        gpioWrite(args->pin_trig, HIGH);
        usleep(10); /* microseconds */
        gpioWrite(args->pin_trig, LOW);

        /* Wait until the echo pin gets pulled up */
        clock_gettime(CLOCK_REALTIME, &timeout_start);
        while (gpioRead(args->pin_echo) == 0 && !*(args->p_terminate)) {
            clock_gettime(CLOCK_REALTIME, &timeout_now);
            if (timeout_now.tv_nsec - timeout_start.tv_nsec > timeout_max) { 
                valid_reading = false;
                break; 
            }

        }
        /* Echo pin is HIGH: Start waiting to receive a signal */
        clock_gettime(CLOCK_REALTIME, &start_time);
        clock_gettime(CLOCK_REALTIME, &timeout_start);
        while (gpioRead(args->pin_echo) == 1 && !*(args->p_terminate)) {
            clock_gettime(CLOCK_REALTIME, &timeout_now);
            if (timeout_now.tv_nsec - timeout_start.tv_nsec > timeout_max) { 
                valid_reading = false;
                break; 
            }
        }

        /* Signal has been received and echo pin is low again. End timer */
        clock_gettime(CLOCK_REALTIME, &end_time);
        time_elapsed_ns = end_time.tv_nsec - start_time.tv_nsec;

        /* Calculate and display distance based on the time elapsed */
        args->distance = valid_reading ? distance_m(time_elapsed_ns) * 100.0f : -1.0f;
        update_reading(args);

        usleep(150000);
    }
}

void update_reading(SonarArgs* args)
{
    //printf("FRONT: %3.2f \t LEFT %3.2f\n", sonar_args_front->distance, sonar_args_left->distance);
    if (args->distance > 0.0f && args->distance < args->max_distance) {
        if (args->confidence < 100) {
            args->confidence += 1;
        }
    }
    else {
        if (args->confidence > 0) {
            args->confidence -= 1;
        }
    }
}

bool object_present(SonarArgs* args)
{
    return args->confidence >= 5;
}