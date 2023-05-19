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
#define FRONTVIEW_LEFT 315.0
#define FRONTVIEW_RIGHT 45.0

#define LEFTVIEW_LEFT 180.0
#define LEFTVIEW_RIGHT 0.0

#define RIGHTVIEW_LEFT 0.0
#define RIGHTVIEW_RIGHT 180.0

#define OBSTACLE_DISTANCE 100.0

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


#endif  /* _MOVEMENT_H */
