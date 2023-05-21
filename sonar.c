#include <math.h>   /* fabs() */
#include "sonar.h"

void init_SonarArgs(SonarArgs* args, uint8_t pin_trig, uint8_t pin_echo)
{
    args->distance_cm = 0.0f;
    args->last_distance_cm = 0.0f;
    args->confidence = 0;
    args->pin_trig = pin_trig;
    args->pin_echo = pin_echo;
    args->p_terminate = NULL;
}

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
    return 0.5f * (float)VSOUND_M_NS * time_ns;
}

float distance_cm(time_t time_ns)
{
    return distance_m(time_ns) * 100.0f;
}

/**
 * Increment confidence, not to exceed CONFIDENCE_MAX
 */
static void increment_confidence(SonarArgs* args)
{
    if (args->confidence < SONAR_CONFIDENCE_MAX) { 
        ++args->confidence; 
    }
}

/**
 * Decrement confidence based on a delta value.
 * The confidence level will be decreased by delta,
 * or will hit a floor of 0.
 */
static void decrement_confidence(SonarArgs* args, int delta)
{
    if (args->confidence < (int)delta) {
        args->confidence = 0;
    } 
    else {
        args->confidence -= (int) delta;
    }
}

void* watch_sonar(SonarArgs* args)
{
    struct timespec start_time;     /* Time when trigger signal was sent */
    struct timespec end_time;       /* Time when echo signal was received */
    time_t time_elapsed_ns = 0;     /* Time spent waiting for echo signal */

    /* Create a timer to break out of infinite loops caused by bad readings */
    struct timespec timeout;        
    struct timespec now;       
    time_t timeout_ns = (time_t)SONAR_TIMEOUT_NS;

    /* Delta is used to determine the distance between two successive readings.
     * If the delta between two readings exceeds the threshold, the confidence
     * will be reduced by delta * delta_weight. Otherwise confidence is incremented. */
    int delta = 0;              /* Distance between readings */
    int delta_weight = 2;       /* Adjust the impact of delta in confidence reduction */
    int delta_threshold = 5;    /* Maximum delta for confidence increase. */

    uint8_t consecutive_bad_readings = 0;
    bool valid_reading;         /* Indicates that a timeout occurred and reading failed. */

    /* Continue polling the sensor until termination occurs. */
    while (!*(args->p_terminate))
    {
        valid_reading = true;

        /* Send a signal for 10 microseconds */
        gpioWrite(args->pin_trig, HIGH);
        usleep(10); /* microseconds */
        gpioWrite(args->pin_trig, LOW);

        /* Wait until the echo pin gets pulled up */
        clock_gettime(CLOCK_REALTIME, &timeout);
        while (gpioRead(args->pin_echo) == 0 && valid_reading && !*(args->p_terminate)) {
            clock_gettime(CLOCK_REALTIME, &now);
            valid_reading = now.tv_nsec - timeout.tv_nsec < timeout_ns;
        }
        /* Echo pin is HIGH: Start waiting to receive a signal */
        clock_gettime(CLOCK_REALTIME, &start_time);
        clock_gettime(CLOCK_REALTIME, &timeout);
        while (gpioRead(args->pin_echo) == 1 && valid_reading && !*(args->p_terminate)) {
            clock_gettime(CLOCK_REALTIME, &now);
            valid_reading = now.tv_nsec - timeout.tv_nsec < timeout_ns;
        }
        if (!valid_reading) 
        {
            /* Don't bother calculating distance on a bad reading. */
            if (consecutive_bad_readings < 100) {
                ++consecutive_bad_readings;
            }
            decrement_confidence(args, consecutive_bad_readings * delta_weight);
        }
        else  
        {
            consecutive_bad_readings = 0;
            /* Signal has been received and echo pin is low again. End timer */
            clock_gettime(CLOCK_REALTIME, &end_time);
            time_elapsed_ns = end_time.tv_nsec - start_time.tv_nsec;

            /* Calculate distance based on the time elapsed */
            args->distance_cm = distance_cm(time_elapsed_ns);

            /* Update confidence based on the difference between current and last reading */
            delta = (int) fabs(args->distance_cm - args->last_distance_cm) * delta_weight;
            if (args->distance_cm > 0 && delta < delta_threshold) {
                increment_confidence(args);
            } 
            else  {
                decrement_confidence(args, (int)delta);
            }
            args->last_distance_cm = args->distance_cm;
        }
        /* Delay to limit polling frequency */
        usleep(SONAR_PING_DELAY_US);
    }
}

bool object_present(SonarArgs args, float max_distance_cm)
{
    return (args.distance_cm <= max_distance_cm 
        && args.confidence >= SONAR_CONFIDENCE_THRESHOLD);
}