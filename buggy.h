#ifndef BUGGY_H
#define BUGGY_H

#include "motor.h"

// Struct tying together both motor of the buggy for passing to movement functions.
struct Buggy {
	Motor motor_a;
	Motor motor_b;
};

// Move buggy a specified distance in a specified direction.
void move_buggy(Buggy* buggy, Direction dir, float meters);
// Rotate buggy a specified angle in the clockwise direction.
void rotate_buggy(Buggy* buggy, float degrees);

#endif