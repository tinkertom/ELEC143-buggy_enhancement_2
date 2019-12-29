#include "motor.h"

void calibrate_motor_dir(Motor* motor)
{
	motor->pwm.period_ms(10);
	motor->dir.write(FORWARD);
	motor->pwm.write(1.0f);
	
	bool calibrated = false;
	int hall_pair_state = motor->hall_pair.read();
	while (!calibrated) {
		if (hall_pair_state == LOW_LOW && motor->hall_pair.read() == LOW_HIGH) {
			motor->inverted = FORWARD;
			calibrated = true;
		} else if (hall_pair_state == LOW_LOW && motor->hall_pair.read() == HIGH_LOW) {
			motor->inverted = REVERSE;
			calibrated = true;
		}
		hall_pair_state = motor->hall_pair.read();
	}
	
	motor->pwm.write(0.0f);
}

bool poll_motor(BusIn* hall_pair, int* prev_hall_pair_state, Direction direction, Direction inverted,  Timer* timer, int hall_timing[2][2])
{
	int curr_hall_pair_state = hall_pair->read();

	if (inverted) {
		direction = (Direction)!direction;
	}
	
	if (direction == FORWARD) {
		if (*prev_hall_pair_state == LOW_LOW && curr_hall_pair_state == LOW_HIGH) {
			*prev_hall_pair_state = curr_hall_pair_state;
			hall_timing[0][0] = timer->read_us();
		} else if (*prev_hall_pair_state == LOW_HIGH && curr_hall_pair_state == HIGH_HIGH) {
			*prev_hall_pair_state = curr_hall_pair_state;
			hall_timing[1][0] = timer->read_us();
		} else if (*prev_hall_pair_state == HIGH_HIGH && curr_hall_pair_state == HIGH_LOW) {
			*prev_hall_pair_state = curr_hall_pair_state;
			hall_timing[0][1] = timer->read_us();
		} else if (*prev_hall_pair_state == HIGH_LOW && curr_hall_pair_state == LOW_LOW) {
			*prev_hall_pair_state = curr_hall_pair_state;
			hall_timing[1][1] = timer->read_us();
			return true;
		}
	} else {
		if (*prev_hall_pair_state == LOW_LOW && curr_hall_pair_state == HIGH_LOW) {
			*prev_hall_pair_state = curr_hall_pair_state;
			hall_timing[0][0] = timer->read_us();
		} else if (*prev_hall_pair_state == HIGH_LOW && curr_hall_pair_state == HIGH_HIGH) {
			*prev_hall_pair_state = curr_hall_pair_state;
			hall_timing[1][0] = timer->read_us();
		} else if (*prev_hall_pair_state == HIGH_HIGH && curr_hall_pair_state == LOW_HIGH) {
			*prev_hall_pair_state = curr_hall_pair_state;
			hall_timing[0][1] = timer->read_us();
		} else if (*prev_hall_pair_state == LOW_HIGH && curr_hall_pair_state == LOW_LOW) {
			*prev_hall_pair_state = curr_hall_pair_state;
			hall_timing[1][1] = timer->read_us();
			return true;
		}
	}
	
	return false;
}

float get_adjusted_duty(float duty, int hall_timing[2][2])
{
	// Calculate period of one pulse.
	float p1 = 2.0f * (hall_timing[0][1] - hall_timing[0][0]);
	float p2 = 2.0f * (hall_timing[1][1] - hall_timing[1][0]);
	// Take absolute value of period result to accomodate motor moving in opposite direction.
	/*if (p1 < 0.0f)
		p1 *= -1;
	if (p2 < 0.0f)
		p2 *= -1;*/
	// Calculate frequency of motor rotation.
	// frequency = 1 / average of both periods * pulses per motor revolution (then converted from microseconds to seconds)
	float mf = 1.0f / ((p1 + p2) * 0.5f * (float)3.0E-6);
	// Calculate required change in duty cycle.
	// delta = 1 - wheel frequency, wheel frequency = motor frequency / motor to wheel ratio
	float d = 1.0f - mf / 20.8f;
	
	duty = duty + d * 0.1f;
	
	// Make sure the outputed value are within the range of what should be writen to the pwm pins.
	if (duty > 1.0f)
		duty = 1.0f;
	else if (duty < 0.05f)
		duty = 0.05f;
	
	return duty;
}

void control_motors(Motor* motor_a, Motor* motor_b, int pulses)
{
	// Set initial duty cycle. Will be adjusted to achieve one wheel revolution per second.
	float duty_a = 0.5f, duty_b = 0.5f;
	int hall_pair_state_a = LOW_LOW, hall_pair_state_b = LOW_LOW;
	int hall_timing_a[2][2], hall_timing_b[2][2];
	int pulse_count_a = 0, pulse_count_b = 0;
	// Timer object for hall effect sensor timings.
	Timer timer;
	timer.start();
	
	// Set motors to initial duty cycle.
	motor_a->pwm.write(duty_a);
	motor_b->pwm.write(duty_b);
	
	// Main while loop that handles pulse counting, and duty cycle adjustment.
	while (pulse_count_a < pulses || pulse_count_b < pulses) {
		if (pulse_count_a < pulses) {
			if (poll_motor(&motor_a->hall_pair, &hall_pair_state_a, (Direction)motor_a->dir.read(), motor_a->inverted, &timer, hall_timing_a)) {
				duty_a = get_adjusted_duty(duty_a, hall_timing_a);
				motor_a->pwm.write(duty_a);
				++pulse_count_a;
			}
		}
		if (pulse_count_b < pulses) {
			if (poll_motor(&motor_b->hall_pair, &hall_pair_state_b, (Direction)motor_b->dir.read(), motor_b->inverted, &timer, hall_timing_b)) {
				duty_b = get_adjusted_duty(duty_b, hall_timing_b);
				motor_b->pwm.write(duty_b);
				++pulse_count_b;
			}
		}
	}
	// Set motors to stop.
	motor_a->pwm.write(0.0f);
	motor_b->pwm.write(0.0f);
	timer.stop();
}