#ifndef BUGGY_H
#define BUGGY_H

#include "motor.h"

struct Buggy {
	Motor motor_a;
	Motor motor_b;
};

void move_buggy(Buggy* buggy, Direction dir, float meters);
void rotate_buggy(Buggy* buggy, float degrees);

#endif
