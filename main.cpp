#include "mbed.h"

#define WHEEL_CIRCUM 176 // Millimeters
#define PULSES_PER_REV 62 // Pulses per wheel revolution
#define PULSES_PER_METER 1000 / WHEEL_CIRCUM * PULSES_PER_REV
#define TURNING_CIRCLE 124 * 3.14 // Turning circle in millimeters 
#define PULSES_PER_TURN TURNING_CIRCLE / WHEEL_CIRCUM * PULSES_PER_REV

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
};

struct Buggy {
	Motor motor_a;
	Motor motor_b;
};

bool poll_motor(BusIn* hall_pair, int* prev_hall_pair_state, Timer* timer, int hall_timing[2][2]);
float get_adjusted_duty(float duty, int hall_timings[2][2]);
void control_motors(PwmOut* pwm_a, PwmOut* pwm_b, BusIn* hall_pair_a, BusIn* hall_pair_b, int pulses);
void move_buggy(Buggy* buggy, Direction dir, float meters);
void rotate_buggy(Buggy* buggy, float degrees);

Serial terminal(USBTX, USBRX);

int main()
{
	// Debugging serial connection
	terminal.baud(115200);
	
	// Indication LED
	DigitalOut led(LED1);
	// User Switch
	DigitalIn user_switch(USER_BUTTON);
	
	Buggy buggy = {{PA_8, PA_9, {PB_2, PB_1}}, {PB_4, PB_10, {PB_15, PB_14}}};
	
	// Buggy setup
	// Set motor period to 100Hz.
	buggy.motor_a.pwm.period_ms(10);
	buggy.motor_b.pwm.period_ms(10);
	// Set initial buggy direction.
	buggy.motor_a.dir = FORWARD;
	buggy.motor_b.dir = FORWARD;
	// Make sure buggy is stationary by setting both motor to stop.
	buggy.motor_a.pwm.write(0.0f);
	buggy.motor_b.pwm.write(0.0f);
	
	float duty = 0.5f;
	int hall_pair_state = 0;
	int hall_timing[2][2];
	Timer timer;
	
	while (true) {
		led = 0;
		while (user_switch == 1);
		led = 1;
		// Wait for finger to clear from buggy.
		wait_us(5000);
		
		
		
		timer.reset();
		timer.start();
		while(!poll_motor(&buggy.motor_a.hall_pair, &hall_pair_state, &timer, hall_timing));
		duty = get_adjusted_duty(duty, hall_timing);
		timer.stop();
		terminal.printf("hall_timing[0][0] = %d\n\r", hall_timing[0][0]);
		terminal.printf("hall_timing[1][0] = %d\n\r", hall_timing[1][0]);
		terminal.printf("hall_timing[0][1] = %d\n\r", hall_timing[0][1]);
		terminal.printf("hall_timing[1][1] = %d\n\r", hall_timing[1][1]);
		//terminal.printf("Average motor freq: %6.2fHz \t Wheel freq: %6.2f\n\r", fa, wa);
		terminal.printf("Ajusted duty cycle: %6.2f\n\r", duty);
		
		
		//move_buggy(&buggy, FORWARD, 3.0f);
		//rotate_buggy(&buggy, 360.0f);
		/*move_buggy(&buggy, FORWARD, 1.0f);
		wait_us(5000);
		rotate_buggy(&buggy, 153.4f);
		wait_us(5000);
		move_buggy(&buggy, FORWARD, 1.12f);
		wait_us(5000);
		rotate_buggy(&buggy, 116.6f);
		wait_us(5000);
		move_buggy(&buggy, FORWARD, 0.5f);
		wait_us(5000);
		rotate_buggy(&buggy, 90.0f);*/
	}
}

bool poll_motor(BusIn* hall_pair, int* prev_hall_pair_state, Timer* timer, int hall_timing[2][2])
{
	int curr_hall_pair_state = hall_pair->read();
	
	if (*prev_hall_pair_state == LOW_LOW && (curr_hall_pair_state == HIGH_LOW || curr_hall_pair_state == LOW_HIGH)) {
		*prev_hall_pair_state = curr_hall_pair_state;
		hall_timing[0][0] = timer->read_us();
	} else if ((*prev_hall_pair_state == LOW_HIGH || *prev_hall_pair_state == HIGH_LOW) && curr_hall_pair_state == HIGH_HIGH) {
		*prev_hall_pair_state = curr_hall_pair_state;
		hall_timing[1][0] = timer->read_us();
	} else if (*prev_hall_pair_state == HIGH_HIGH && (curr_hall_pair_state == HIGH_LOW || curr_hall_pair_state == LOW_HIGH)) {
		*prev_hall_pair_state = curr_hall_pair_state;
		hall_timing[0][1] = timer->read_us();
	} else if ((*prev_hall_pair_state == LOW_HIGH || *prev_hall_pair_state == HIGH_LOW) && curr_hall_pair_state == LOW_LOW) {
		*prev_hall_pair_state = curr_hall_pair_state;
		hall_timing[1][1] = timer->read_us();
		return true;
	}
	
	return true;
}

float get_adjusted_duty(float duty, int hall_timing[2][2])
{
	// Calculate period of one pulse.
	float p1 = 2.0f * (hall_timing[0][1] - hall_timing[0][0]);
	float p2 = 2.0f * (hall_timing[1][1] - hall_timing[1][0]);
	// Take absolute value of period result to accomodate motor moving in opposite direction.
	if (p1 < 0.0f)
		p1 *= -1;
	if (p2 < 0.0f)
		p2 *= -1;
	// Calculate frequency of motor rotation.
	// frequency = 1 / average of both periods * pulses per motor revolution (then converted from milliseconds to seconds)
	float mf = 1.0f / (p1 + p2) * 0.5f * 3.0E-6;
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

void control_motors(PwmOut* pwm_a, PwmOut* pwm_b, BusIn* hall_pair_a, BusIn* hall_pair_b, int pulses)
{
	float duty_a = 0.5f, duty_b = 0.5f;
	int hall_pair_state_a = 0, hall_pair_state_b = 0;
	int hall_timing_a[2][2], hall_timing_b[2][2];
	int pulse_count_a = 0, pulse_count_b = 0;
	Timer timer;
	
	pwm_a->write(duty_a);
	pwm_b->write(duty_b);
	timer.start();
	while (pulse_count_a < pulses || pulse_count_b < pulses) {
		if (pulse_count_a < pulses) {
			if (poll_motor(hall_pair_a, &hall_pair_state_a, &timer, hall_timing_a)) {
				duty_a = get_adjusted_duty(duty_a, hall_timing_a);
				pwm_a->write(duty_a);
				++pulse_count_a;
			}
		}
		if (pulse_count_b < pulses) {
			if (poll_motor(hall_pair_b, &hall_pair_state_b, &timer, hall_timing_b)) {
				duty_b = get_adjusted_duty(duty_b, hall_timing_b);
				pwm_b->write(duty_b);
				++pulse_count_b;
			}
		}
	}
	pwm_a->write(0.0f);
	pwm_b->write(0.0f);
	timer.stop();
}

void move_buggy(Buggy* buggy, Direction dir, float meters)
{
	int pulses = meters * PULSES_PER_METER;
	
	buggy->motor_a.dir.write(dir);
	buggy->motor_b.dir.write(dir);
	control_motors(&buggy->motor_a.pwm, &buggy->motor_b.pwm, &buggy->motor_a.hall_pair, &buggy->motor_b.hall_pair, pulses);
}

void rotate_buggy(Buggy* buggy, float degrees)
{
	unsigned int pulses = PULSES_PER_TURN / 360 * degrees;
	
	buggy->motor_a.dir.write(FORWARD);
	buggy->motor_b.dir.write(REVERSE);
	control_motors(&buggy->motor_a.pwm, &buggy->motor_b.pwm, &buggy->motor_a.hall_pair, &buggy->motor_b.hall_pair, pulses);
}