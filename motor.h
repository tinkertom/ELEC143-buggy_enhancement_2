#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"

// Enum used indicate the direction of the buggy itself, and whether a motors physical direction in the buggy is inverted.
enum Direction {
	FORWARD = 0,
	REVERSE = 1,
};

// Enum type used for comparing against the state of the hall effect sensor bus.
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
	// Physical direction of the motor in the buggy.
	Direction inverted;
};

// Motor related function definitions.
// Detects whether the motor move in the expected direction when the direction pin is set to forward, writing the result to the motor struct. 
void calibrate_motor_dir(Motor* motor);
// Compares the current state of the motor with the previous state, and return 'true' if a full pulse sequence has been detected.
bool poll_motor(BusIn* hall_pair, int* prev_hall_pair_state, Direction direction, bool inverted,  Timer* timer, int hall_timing[2][2]);
// Calculates the speed of the of the motor, and then returns an adjusted duty cycle to achieve one wheel revolution per second.
float get_adjusted_duty(float duty, int hall_timings[2][2]);
// Move each motor for a specific amount of pulses, while maintaining constant speed across both motors.
void control_motors(Motor* motor_a, Motor* motor_b, int pulses);

#endif