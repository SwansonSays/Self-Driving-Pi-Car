#include "movement.h"
#include "sensor.h"


/**
 * Turn left by increasing the speed of the right motor, and 
 * decreasing the speed of the left. Turning will not be performed
 * unless the sensor reading confidence threshold has been met. 
 *
 * The confidence threshold is incremented with each successive
 * request to turn in the same direction, and is reset to zero
 * when the requested direction changes.
 */
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

/**
 * Turn right by decreasing the speed of the right motor, and 
 * increasing the speed of the left. Turning will not be performed
 * unless the sensor reading confidence threshold has been met. 
 *
 * The confidence threshold is incremented with each successive
 * request to turn in the same direction, and is reset to zero
 * when the requested direction changes.
 */
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


/** 
 * Straighten out the car by setting both wheels to the same speed (100%).

 * Adjustment will not be performed unless the sensor reading confidence 
 * threshold has been met. 
 *
 * The confidence threshold is incremented with each successive
 * request to go straight, and is reset to zero when the requested 
 * direction changes.
 */
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


/**
 * Evaluate the line sensor values and take the appropriate action 
 * to follow the line.
 */
void follow_line(uint8_t line_sensor_vals[], ProgramState* state)
{
    /* IMPORTANT: The sensor evaluations must be performed in exactly this order.
     * (from most specific, to least). 
     *
     * First check all three sensors, then pairs of sensors, then finally 
     * cases where only one sensor is checked. 
     *
     * If the order is changed so that single sensors are checked first, it will 
     * short circuit the evaluation and cases where more than one sensor are 
     * active will get skipped over.
     */

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
    else  {
        /* Fall back on the last attempted direction */
        if (state->last_dir == RIGHT) {
            turn_right(state);
        }
        else if (state->last_dir == LEFT) {
            turn_left(state);
        }
    }
}

