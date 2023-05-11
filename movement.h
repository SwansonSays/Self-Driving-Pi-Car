#ifndef _MOVEMENT_H
#define _MOVEMENT_H


#include <stdint.h>  /* uint8_t */
#include "definitions.h"
#include "MotorDriver.h"

#define MOTOR_LEFT  MOTORB
#define MOTOR_RIGHT MOTORA


#define CONFIDENCE_THRESHOLD 8
#define CONFIDENCE_MAX 100

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
    uint8_t inner_confidence;
    uint8_t outer_confidence;
} ProgramState;

void turn_left(ProgramState* state, uint8_t* confidence);
void turn_right(ProgramState* state, uint8_t* confidence);
void go_straight(ProgramState* state, uint8_t* confidence);


#endif  /* _MOVEMENT_H */
