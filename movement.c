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
* File:         movement.c
*
* Description:
*   Implementation for movement related code, including line following and
*   obstacle avoidance.
******************************************************************************/


#include <time.h>

#include "movement.h"
#include "sensor.h"


/**
 * Helper function to increment a confidence value
 * without exceeding the maximum.
 */
void increment_confidence(uint8_t* confidence)
{
    if (*confidence < CONFIDENCE_MAX) {
        ++(*confidence);
    }
}

/**
 * Turn left by increasing the speed of the right motor, and 
 * decreasing the speed of the left. Turning will not be performed
 * unless the sensor reading confidence threshold has been met. 
 *
 * The confidence threshold is incremented with each successive
 * request to turn in the same direction, and is reset to zero
 * when the requested direction changes.
 */
void turn_left(ProgramState* state, uint8_t* confidence)
{
    //printf("Turning LEFT\n");
    if (state->last_req == LEFT)
    {
        increment_confidence(confidence);
        if (*confidence >= CONFIDENCE_THRESHOLD)
        {
            state->speed_left = Motor_Decrease_Speed(MOTOR_LEFT, state->speed_left, state->speed_left - 5, 1);
            state->speed_right = Motor_Increase_Speed(MOTOR_RIGHT, state->speed_right, state->speed_right + 5, 1);
            state->last_dir = LEFT;
        }
    }
    else {
        *confidence = 0;
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
void turn_right(ProgramState* state, uint8_t* confidence)
{
    //printf("Turning RIGHT\n");
    if (state->last_req == RIGHT) 
    {
        increment_confidence(confidence);
        if (*confidence >= CONFIDENCE_THRESHOLD)
        {
            state->speed_left = Motor_Increase_Speed(MOTOR_LEFT, state->speed_left, state->speed_left + 5, 1);
            state->speed_right = Motor_Decrease_Speed(MOTOR_RIGHT, state->speed_right, state->speed_right - 5, 1);
            state->last_dir = RIGHT;
        }
    }
    else { 
        *confidence = 0;
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
void go_straight(ProgramState* state, uint8_t* confidence)
{
    //printf("Turning RIGHT\n");
    if (state->last_req == STRAIGHT) 
    {
        if (*confidence < CONFIDENCE_MAX) {
            ++(*confidence);
        }
        if (*confidence >= CONFIDENCE_THRESHOLD)
        {
            state->speed_left = Motor_Increase_Speed(MOTOR_LEFT, state->speed_left, 100, 5);
            state->speed_right = Motor_Increase_Speed(MOTOR_RIGHT, state->speed_right, 100, 5);
            state->last_dir = STRAIGHT;
        }
    }
    else { 
        *confidence = 0;
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
    /* Right and outer-right */
    if (line_sensor_vals[3] == HIGH && line_sensor_vals[4] == LOW)
    {
        turn_left(state, &(state->outer_confidence));
    }
    /* Left and outer-left */
    else if (line_sensor_vals[3] == LOW && line_sensor_vals[4] == HIGH)
    {
        turn_right(state, &(state->outer_confidence));
    }
    else {
        state->outer_confidence = 0;

        /* (1, 1, 1) All three sensors are on */
        if (line_sensor_vals[0] == HIGH && line_sensor_vals[1] == HIGH && line_sensor_vals[2] == HIGH) {
        /* Keep performing the current action */
        }
        /* (1, 1, 0) LEFT and CENTER */
        else if (line_sensor_vals[0] == HIGH && line_sensor_vals[1] == HIGH) {
            turn_left(state, &(state->inner_confidence));
        }
        /* (0, 1, 1) CENTER and RIGHT */
        else if (line_sensor_vals[1] == HIGH && line_sensor_vals[2] == HIGH) {
            turn_right(state, &(state->inner_confidence));
        }
        /* (1, 0, 1) LEFT and RIGHT */
        else if (line_sensor_vals[0] == HIGH && line_sensor_vals[2] == HIGH) {
        /* Keep performing the current action */
        }
        /* (1, 0, 0) LEFT only */
        else if (line_sensor_vals[0] == HIGH) {
            turn_left(state, &(state->inner_confidence));
        }
        /* (0, 1, 0) CENTER only */
        else if (line_sensor_vals[1] == HIGH) {
            go_straight(state, &(state->inner_confidence));
        }
        /* (0, 0, 1) RIGHT only */
        else if (line_sensor_vals[2] == HIGH) {
            turn_right(state, &(state->inner_confidence));
        }
        /* (0, 0, 0) no sensors active */
        else  {
            /* Fall back on the last attempted direction */
            if (state->last_dir == RIGHT) {
                turn_right(state, &(state->inner_confidence));
            }
            else if (state->last_dir == LEFT) {
                turn_left(state, &(state->inner_confidence));
            }
        }
    }
}

/**
 * Avoid an obstacle using input from the sonar sensors.
 * Navigate around the obstacle by moving in a large rectangle around the object's bounding region.\
 * 1. Turn right, and move forward until past the object
 * 2. Turn left. Move until the object is detected, and continue until past the object.
 * 3. Turn left. Move forward until the line is deteced.
 * 4. Turn right. Return control to the line following routine.
 *
 * To mitigate errors in the obstacle sensors, several consecutive readings
 * must indicate the presence (or lack of) an obstacle before action is taken.
 * This process is attempted multiple times to account for possible misinterpretations
 * of the sensor readings.
 */
void avoid_obstacle(SonarArgs* args_front, SonarArgs* args_left, ProgramState* state, uint8_t line_sensor_vals[])
{
    float left_distance = 40.0f;

    int confidence = 0;
    int confidence_threshold = 50;

    bool reading = true;
    bool last_reading = true;

    int attempts = 3;

    turn_90(state, RIGHT);

    /* Go forward as long as there is an object to the left */
    for (int i = 0; i < attempts; i++)
    {
        confidence = 0;
        while (!*(args_left->p_terminate)) {
            reading = object_present(args_left, left_distance);
            if (reading == last_reading) {
                if (confidence < 100) {
                    ++confidence;
                }
            }
            else {
                confidence /= 2;
            }
            last_reading = reading;

            if (reading == false && confidence >= confidence_threshold) {
                break;
            }
            usleep(10000);
        }
    }
    printf("PASSED OBJECT\n");
    turn_90(state, LEFT);

    /* Go forward until the object is not detected to the left */
    for (int i = 0; i < attempts; i++)
    {
        confidence = 0;
        while (!*(args_left->p_terminate)) {
            reading = object_present(args_left, left_distance);
            if (reading == last_reading) {
                if (confidence < 100) {
                    ++confidence;
                }
            }
            else {
                confidence /= 2;
            }
            last_reading = reading;

            if (reading == true && confidence >= confidence_threshold) {
                break;
            }
            usleep(10000);
        }
    }
    /* Go forward as long as there is an object to the left */
    for (int i = 0; i < attempts; i++) 
    {
        confidence = 0;
        while (!*(args_left->p_terminate)) {
            reading = object_present(args_left, left_distance);
            if (reading == last_reading) {
                if (confidence < 100) {
                    ++confidence;
                }
            }
            else {
                confidence /= 2;
            }
            last_reading = reading;

            if (reading == false && confidence >= confidence_threshold) {
                break;
            }
            usleep(10000);
        }
    }

    turn_90(state, LEFT);

    /* Continue forward until the line is detected 
     * (at least two sensors are active) */
    int count = 0;
    while (count < 2 && !*(args_left->p_terminate))
    {
        count = 0;
        for (int i = 0; i < 5; i++) {
            count += line_sensor_vals[i];
        }
    }
    printf("LINE DETECTED\n");
    turn_90(state, RIGHT);
}

/**
 * Initiate a turn in the specified direction by setting the motor 
 * directions with respect to their mounting orientations.
 */
void set_turn_direction(ProgramState* state, DIR dir)
{
    printf("Changing direction ");
    switch (dir)
    {
        case LEFT:
            printf("LEFT\n");
            // turn left motor backwards, right forward
            Motor_Set_Direction(MOTOR_LEFT, MOTOR_LEFT_BACKWARD, 100);
            Motor_Set_Direction(MOTOR_RIGHT, MOTOR_RIGHT_FORWARD, 100);
            break;
        case RIGHT:
            printf("RIGHT\n");
            // turn left motor forward, right backward
            Motor_Set_Direction(MOTOR_LEFT, MOTOR_LEFT_FORWARD, 100);
            Motor_Set_Direction(MOTOR_RIGHT, MOTOR_RIGHT_BACKWARD, 100);
            break;
        case FORWARD:
            printf("FORWARD\n");
            Motor_Set_Direction(MOTOR_LEFT, MOTOR_LEFT_FORWARD, 100);
            Motor_Set_Direction(MOTOR_RIGHT, MOTOR_RIGHT_FORWARD, 100);
            break;
        case BACKWARD:
            printf("BACKWARD\n");
            Motor_Set_Direction(MOTOR_LEFT, MOTOR_LEFT_BACKWARD, 100);
            Motor_Set_Direction(MOTOR_RIGHT, MOTOR_RIGHT_BACKWARD, 100);
            break;
        default:
            break;
    }
}

/**
 * Perform a 90 degree turn in the specified direction.
 */
void turn_90(ProgramState* state, DIR dir)
{
    if (dir == LEFT)
    {
        set_turn_direction(state, dir);
        usleep(1100000);
    }
    else if (dir == RIGHT) 
    {
        set_turn_direction(state, dir);
        usleep(1000000);
    }
    set_turn_direction(state, FORWARD);
}
