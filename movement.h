 /******************************************************************************
* Class:        CSC-615-01 Spring 2023
*
* Names:        Zachary Colbert
*               Sajan Gurung
*               Robert Swanson
*               Tyler Wartzok
*
* Github ID:    ttwartzok
* Project:      Final Project - Self Driving Car 
*
* File:         movement.h
*
* Description:
*   Declarations for movement related code.  
******************************************************************************/


#ifndef _MOVEMENT_H
#define _MOVEMENT_H


#include <stdint.h>  /* uint8_t */
#include <stdbool.h>
#include "sonar.h"
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
#define LEFTVIEW_RIGHT 15.0f

#define RIGHTVIEW_LEFT 0.0f
#define RIGHTVIEW_RIGHT 180.0f

#define OBSTACLE_DISTANCE 475.0f

typedef enum {
    LINE,
    OBSTACLE
} MODE;

typedef struct
{
    DIR last_dir;               /* Last successful direction */
    DIR last_req;               /* Last attempted direction */
    UBYTE speed_left;           /* Speed of left motor */
    UBYTE speed_right;          /* Speed of right motor */
    uint8_t inner_confidence;   /* Confidence for inner sensor direction */
    uint8_t outer_confidence;   /* Confidence for outer sensor direction */
    bool* p_terminate;          /* Termination flag */
} ProgramState;

void turn_left(ProgramState* state, uint8_t* confidence);
void turn_right(ProgramState* state, uint8_t* confidence);
void go_straight(ProgramState* state, uint8_t* confidence);

void set_turn_direction(ProgramState* state, DIR dir);
void turn_90(ProgramState* state, DIR dir);

void avoid_obstacle(SonarArgs* args_front, SonarArgs* args_left, ProgramState* state, uint8_t line_sensor_vals[]);

#endif  /* _MOVEMENT_H */
