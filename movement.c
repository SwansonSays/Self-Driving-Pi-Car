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

/* 
*  Checks the current closest object in params too see if it is within a viewport of left_theta
*  to right_theta and that is closer then max_distance.
*  Returns -1 if scan is invalid
*  Returns 0 if the scan is valid but not within our viewport
*  Returns 1 if scan is valid and within viewport 
*/
int object_in_viewport(struct Params* params, float left_theta, float right_theta, float max_distance)
{
    printf("THETA %f DISTANCE %f AGE %d\n", params->theta, params->distance, params->age);

    /* If the reading is invalid, abort immediately */
    if (params->distance < 1 || params->theta < 0) {
	    printf("theta or distance < 0 aborting\n");
	    return -1; 
    }
    /* Ignore a reading for an object that is too far away */
    if (params->distance > max_distance) {
	    printf("distance > max aborting\n");
        return -1; 
    }
    /* Ignore a reading if the age is greater then max */
    if (params->age > MAX_AGE) {
        printf("age > max aborting\n");
        return -1; 
    }
    /* Account for the overlap across zero degrees */
    if (left_theta > right_theta) {
        return params->theta >= left_theta || params->theta <= right_theta;
    }
    else {
        return params->theta >= left_theta && params->theta <= right_theta;
    }
}

void check_infront(struct Params* params) {
    printf("Checking in Front\n");
	if (object_in_viewport(&params, FRONTVIEW_LEFT, FRONTVIEW_RIGHT, OBSTACLE_DISTANCE)) {
		printf("Obstacle in front motors off!\n");
		Motor_Stop(MOTOR_LEFT);
		Motor_Stop(MOTOR_RIGHT);
    }
    else {
        printf("FORWARD\n");
        Motor_Set_Direction(MOTOR_LEFT, MOTOR_LEFT_FORWARD, 100);
        Motor_Set_Direction(MOTOR_RIGHT, MOTOR_RIGHT_FORWARD, 100);
    }
}

void avoid_obstacle(struct Params* params, ProgramState* state)
{
    printf("Enter Obstacle Avoidance\n");
    /* Turn right to avoid obstacle by defualt */
    turn_90(state, RIGHT);


    printf("OBST THETA %f DISTANCE %f AGE %f\n", params->theta, params->distance, params->age);
    while (object_in_viewport(params, LEFTVIEW_LEFT, LEFTVIEW_RIGHT, OBSTACLE_DISTANCE)/* || object_in_viewport(&params, LEFTVIEW_LEFT, LEFTVIEW_RIGHT, OBSTACLE_DISTANCE) < 0 */) {
        // something to the left
        //check_infront(&params);
        printf("Going Straight\n");
    }

    turn_90(state, LEFT);


    while (object_in_viewport(params, LEFTVIEW_LEFT, LEFTVIEW_RIGHT, OBSTACLE_DISTANCE)/* || object_in_viewport(&params, LEFTVIEW_LEFT, LEFTVIEW_RIGHT, OBSTACLE_DISTANCE) < 0 */) {
        //check_infront(params);
    }

    turn_90(state, LEFT);


    while (/*linesensors not HIGH*/0){
        //check_infront(params);
        printf("Going Straight\n");
    }
    printf("Line Found\n");

    turn_90(state, RIGHT);

    printf("Turning Right\n");

    state->mode = LINE;
    printf("State = LINE\n");
}

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

void turn_90(ProgramState* state, DIR dir)
{
    DIR last_dir = state->last_dir;
    if (dir == LEFT || dir == RIGHT) {
        set_turn_direction(state, dir);
    }
    usleep(1300000);
    set_turn_direction(state, FORWARD);
}
