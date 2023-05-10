#include "movement.h"
#include "sensor.h"


void turn_left(ProgramState* state)
{
    //printf("Turning LEFT\n");
    if (state->last_req == LEFT)
    {
        ++state->confidence;
        if (state->confidence >= CONFIDENCE_THRESHOLD)
        {
            state->speed_left = Motor_Decrease_Speed(MOTOR_LEFT, state->speed_left, state->speed_left - 5, 1);
            state->speed_right = Motor_Increase_Speed(MOTOR_RIGHT, state->speed_right, state->speed_right + 5, 1);
            state->last_dir = LEFT;
        }
    }
    else {
        state->confidence = 0;
    }
    state->last_req = LEFT;
}

void turn_right(ProgramState* state)
{
    //printf("Turning RIGHT\n");
    if (state->last_req == RIGHT) 
    {
        ++state->confidence;
        if (state->confidence >= CONFIDENCE_THRESHOLD)
        {
            state->speed_left = Motor_Increase_Speed(MOTOR_LEFT, state->speed_left, state->speed_left + 5, 1);
            state->speed_right = Motor_Decrease_Speed(MOTOR_RIGHT, state->speed_right, state->speed_right - 5, 1);
            state->last_dir = RIGHT;
        }
    }
    else { 
        state->confidence = 0;
    }
    state->last_req = RIGHT;
}

void go_straight(ProgramState* state)
{
    //printf("Turning RIGHT\n");
    if (state->last_req == STRAIGHT) 
    {
        ++state->confidence;
        if (state->confidence >= CONFIDENCE_THRESHOLD)
        {
            state->speed_left = Motor_Increase_Speed(MOTOR_LEFT, state->speed_left, 100, 5);
            state->speed_right = Motor_Increase_Speed(MOTOR_RIGHT, state->speed_right, 100, 5);
            state->last_dir = STRAIGHT;
        }
    }
    else { 
        state->confidence = 0;
    }
    state->last_req = STRAIGHT;
}


void follow_line(uint8_t line_sensor_vals[], ProgramState* state)
{
    /* (1, 1, 1) All three sensors are on */
    if (line_sensor_vals[0] == HIGH && line_sensor_vals[1] == HIGH && line_sensor_vals[2] == HIGH) {
        /* Keep performing the current action */
    }
    /* (1, 1, 0) LEFT and CENTER */
    else if (line_sensor_vals[0] == HIGH && line_sensor_vals[1] == HIGH) {
        turn_left(state);
    }
    /* (0, 1, 1) CENTER and RIGHT */
    else if (line_sensor_vals[1] == HIGH && line_sensor_vals[2] == HIGH) {
        turn_right(state);
    }
    /* (1, 0, 1) LEFT and RIGHT */
    else if (line_sensor_vals[0] == HIGH && line_sensor_vals[2] == HIGH) {
        /* Keep performing the current action */
    }
    /* (1, 0, 0) LEFT only */
    else if (line_sensor_vals[0] == HIGH) {
        turn_left(state);
    }
    /* (0, 1, 0) CENTER only */
    else if (line_sensor_vals[1] == HIGH) {
        go_straight(state);
    }
    /* (0, 0, 1) RIGHT only */
    else if (line_sensor_vals[2] == HIGH) {
        turn_right(state);
    }
    /* (0, 0, 0) no sensors active */
    else 
    {
        /* Fall back on the last attempted direction */
        if (state->last_dir == RIGHT) {
            turn_right(state);
        }
        else if (state->last_dir == LEFT) {
            turn_left(state);
        }
    }
}

