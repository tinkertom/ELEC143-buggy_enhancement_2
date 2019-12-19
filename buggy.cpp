#include "buggy.h"

#define WHEEL_CIRCUM 176 // Millimeters
#define PULSES_PER_REV 62 // Pulses per wheel revolution
#define PULSES_PER_METER 1000 / WHEEL_CIRCUM * PULSES_PER_REV
#define TURNING_CIRCLE 127.5 * 3.14 // Turning circle in millimeters 
#define PULSES_PER_TURN TURNING_CIRCLE / WHEEL_CIRCUM * PULSES_PER_REV

void move_buggy(Buggy* buggy, Direction dir, float meters)
{
	int pulses = meters * PULSES_PER_METER;
	
	buggy->motor_a.dir.write(dir);
	buggy->motor_b.dir.write(dir);
	control_motors(&buggy->motor_a, &buggy->motor_b, pulses);
}

void rotate_buggy(Buggy* buggy, float degrees)
{
	unsigned int pulses = PULSES_PER_TURN / 360 * degrees;
	
	buggy->motor_a.dir.write(FORWARD);
	buggy->motor_b.dir.write(REVERSE);
	control_motors(&buggy->motor_a, &buggy->motor_b, pulses);
}