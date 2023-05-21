#ifndef _SONAR_H
#define _SONAR_H

#include "sensor.h"


#define VSOUND_M_S 343                          /* velocity of sound in meters per second */
#define VSOUND_M_NS VSOUND_M_S * 0.000000001    /* velocity of sound in meters per nanosecond */
#define VSOUND_CM_NS VSOUND_M_NS * 100          /* velocity of sound in centimeters per nanosecond */

#define DISTANCE_FACTOR 0.5 * VSOUND_M_NS       /* Precompute the constant factor of s=0.5*v*t */
#define INCHES_PER_METER 39.37

#define MIN_DISTANCE 0.02 /* 2 centimeters */
#define MAX_DISTANCE 4.00 /* 4 meters */


typedef struct {
    float distance;
    float max_distance;
    uint8_t pin_echo;
    uint8_t pin_trig;
    bool* p_terminate;
    uint8_t confidence;
} SonarArgs;


float distance_m(time_t time_ns);
void read_sonar(SonarArgs* args);
void update_reading(SonarArgs* args);
bool object_present(SonarArgs* args);


#endif  /* _SONAR_H */
