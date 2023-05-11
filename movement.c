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
void turn_left(ProgramState* state, uint8_t* confidence)
{
    //printf("Turning LEFT\n");
    if (state->last_req == LEFT)
    {
	    if (*confidence < CONFIDENCE_MAX) {
		++(*confidence);
	    }
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
	    if (*confidence < CONFIDENCE_MAX) {
		++(*confidence);
	    }
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

