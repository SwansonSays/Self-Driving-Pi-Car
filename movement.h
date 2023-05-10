#ifndef _MOVEMENT_H
#define _MOVEMENT_H


#include <stdint.h>  /* uint8_t */
#include "definitions.h"
#include "MotorDriver.h"

#define MOTOR_LEFT  MOTORB
#define MOTOR_RIGHT MOTORA


#define CONFIDENCE_THRESHOLD 8

typedef enum 
{
    STRAIGHT = 0,
    LEFT,
    RIGHT
} DIRECTION;

typedef struct
{
    DIRECTION last_dir;
    DIRECTION last_req;
    UBYTE speed_left;
    UBYTE speed_right;
    uint8_t update;
    uint8_t confidence;
} ProgramState;

void turn_left(ProgramState* state);
void turn_right(ProgramState* state);
void go_straight(ProgramState* state);


#endif  /* _MOVEMENT_H */
