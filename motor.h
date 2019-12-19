#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"

enum Direction {
	FORWARD = 0,
	REVERSE = 1,
};

enum HallPairState {
	LOW_LOW = 0,
	LOW_HIGH = 1,
	HIGH_LOW = 2,
	HIGH_HIGH = 3,
};

struct Motor {
	// PWM motor control pin.
	PwmOut pwm;
	// Motor direction pin.
	DigitalOut dir;
	// Motor hall effect sensor pins.
	BusIn hall_pair;
	
	bool inverted;
};

void calibrate_motor_dir(Motor* motor);
bool poll_motor(BusIn* hall_pair, int* prev_hall_pair_state, Direction direction, bool inverted,  Timer* timer, int hall_timing[2][2]);
float get_adjusted_duty(float duty, int hall_timings[2][2]);
void control_motors(Motor* motor_a, Motor* motor_b, int pulses);

#endif