#ifndef _MOVEMENT_H
#define _MOVEMENT_H


#include <stdint.h>  /* uint8_t */
#include "definitions.h"
#include "MotorDriver.h"

#define MOTOR_LEFT  MOTORA
#define MOTOR_RIGHT MOTORB


#define CONFIDENCE_THRESHOLD 8
#define CONFIDENCE_MAX 100

/* Angles of view if checking for obstacle in front, left, or right */
#define FRONTVIEW_LEFT 315.0f
#define FRONTVIEW_RIGHT 45.0f

#define LEFTVIEW_LEFT 180.0f
#define LEFTVIEW_RIGHT 0.0f

#define RIGHTVIEW_LEFT 0.0f
#define RIGHTVIEW_RIGHT 180.0f

#define OBSTACLE_DISTANCE 100.0f

typedef enum
{
    STRAIGHT = 0,
    LEFT,
    RIGHT
} DIRECTION;

typedef enum {
    LINE,
    OBSTACLE
} MODE;

typedef struct
{
    DIRECTION last_dir;
    DIRECTION last_req;
    MODE mode;
    UBYTE speed_left;
    UBYTE speed_right;
    uint8_t inner_confidence;
    uint8_t outer_confidence;
} ProgramState;

void turn_left(ProgramState* state, uint8_t* confidence);
void turn_right(ProgramState* state, uint8_t* confidence);
void go_straight(ProgramState* state, uint8_t* confidence);
bool object_in_viewport(struct Params* params, float left_theta, float right_theta, float max_distance);

#endif  /* _MOVEMENT_H */
