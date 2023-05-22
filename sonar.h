#ifndef _SONAR_H
#define _SONAR_H

#include "sensor.h"


#define US_PER_S 1000000.0                      /* 10^6 us per second */
#define NS_PER_S 1000000000.0                   /* 10^9 ns per second */

#define VSOUND_M_S 343.0                        /* velocity of sound in meters per second */
#define VSOUND_M_NS VSOUND_M_S / NS_PER_S       /* velocity of sound in meters per nanosecond */
#define VSOUND_CM_NS VSOUND_M_NS * 100.0        /* velocity of sound in centimeters per nanosecond */

#define DISTANCE_FACTOR 0.5 * VSOUND_M_NS       /* Precompute the constant factor of s=0.5*v*t */
#define INCHES_PER_METER 39.37

#define SONAR_MIN_DISTANCE_M 0.02               /* Min range of sensor: 2 centimeters */
#define SONAR_MAX_DISTANCE_M 3.0                /* Max range of sensor: 3 meters */
#define SONAR_MAX_DISTANCE_CM SONAR_MAX_DISTANCE_M * 100.0

/* Establish a timeout to prevent waiting for a reading
 * for more than the longest possible valid range. */
#define SONAR_TIMEOUT_S SONAR_MAX_DISTANCE_M / VSOUND_M_S
#define SONAR_TIMEOUT_NS SONAR_TIMEOUT_S * NS_PER_S

/* Set a limit on how frequently the sensor is polled */
#define SONAR_PING_HZ     20  /* Number of polls per second */
#define SONAR_PING_DELAY_US US_PER_S / SONAR_PING_HZ

#define SONAR_CONFIDENCE_MAX 100
#define SONAR_CONFIDENCE_THRESHOLD 5

typedef struct {
    float distance_cm;         /* the actual distance reading */
    int confidence;
    uint8_t pin_trig;
    uint8_t pin_echo;
    bool* p_terminate;
} SonarArgs;


void init_SonarArgs(SonarArgs* args, uint8_t pin_trig, uint8_t pin_echo);

float distance_m(time_t time_ns);
float distance_cm(time_t time_ns);

void* watch_sonar(SonarArgs* args);
bool object_present(SonarArgs args, float max_distance);


#endif  /* _SONAR_H */
