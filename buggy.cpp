#include "buggy.h"

// Circumference of the a buggy wheel in millimeters.
#define WHEEL_CIRCUM 176
// Motor pulses per wheel revolution.
#define PULSES_PER_REV 62 
// Motor pulses required to travel one meter.
#define PULSES_PER_METER 1000 / WHEEL_CIRCUM * PULSES_PER_REV
// The Circumference of the buggy's turning circle.
#define TURNING_CIRCLE 127.5 * 3.14
// The pulses required for a wheel to travel the circumference of the buggy's turning circle.
#define PULSES_PER_TURN TURNING_CIRCLE / WHEEL_CIRCUM * PULSES_PER_REV

void move_buggy(Buggy* buggy, Direction dir, float meters)
{
	// The amount of pulses required to travel the specified distance.
	int pulses = meters * PULSES_PER_METER;
	
	// Set the buggy's motors to travel in the specified direction.
	buggy->motor_a.dir.write(dir);
	buggy->motor_b.dir.write(dir);
	
	control_motors(&buggy->motor_a, &buggy->motor_b, pulses);
}

void rotate_buggy(Buggy* buggy, float degrees)
{
	// PULSES_PER_TURN/360 gives the amount of pulses to required to rotate one degree.
	// The product of that multiplied by the specified angle will give the pulses required to rotate the desired angle.
	unsigned int pulses = PULSES_PER_TURN / 360 * degrees;
	
	// Set the buggys wheels to travel in seperate directions, as to rotate on the spot.
	buggy->motor_a.dir.write(FORWARD);
	buggy->motor_b.dir.write(REVERSE);
	
	control_motors(&buggy->motor_a, &buggy->motor_b, pulses);
}