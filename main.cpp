#include "mbed.h"
#include "buggy.h"

int main()
{
	// Debugging serial connection.
	Serial terminal(USBTX, USBRX);
	terminal.baud(115200);
	
	// Indication LED.
	DigitalOut led(LED1);
	// User control switch.
	DigitalIn user_switch(USER_BUTTON);
	
	// Declare buggy struct, passing all the necessary pin names.
	//             |left motor______________|  |right motor________________|
	//             |PWM pin|                   |PWM pin|
	//             |     |direction pin|       |     |direction pin|
	//             |            |hall sensor pins|   |       |hall sensor pins|
	Buggy buggy = {{PA_8, PA_9, {PB_2, PB_1}}, {PB_4, PB_10, {PB_15, PB_14}}};
	
	// Calibrate motors.
	terminal.printf("press button to calibrate\n\r");
	while (user_switch == 1);
	calibrate_motor(&buggy.motor_a);
	calibrate_motor(&buggy.motor_b);
	terminal.printf("motor_a inversion = %d\n\r", buggy.motor_a.inverted);
	terminal.printf("motor_b inversion = %d\n\r", buggy.motor_b.inverted);
	// LED1 indicates that buggy has been calibrated.
	led = 1;
	while (user_switch == 0);
	wait_us(500000);
	
	// Make sure buggy is stationary by setting both motor to stop.
	buggy.motor_a.pwm.write(0.0f);
	buggy.motor_b.pwm.write(0.0f);
	
	
	while (true) {
		while (user_switch == 1);
		// Wait for finger to clear from buggy.
		wait_us(500000);

		// Buggy triangle sequence instructions.
		// Travel forward, wiat, rotate, and repeat.
		move_buggy(&buggy, FORWARD, 1.7f);
		wait_us(5000);
		rotate_buggy(&buggy, 144.0f);
		wait_us(5000);
		move_buggy(&buggy, FORWARD, 2.04f);
		wait_us(5000);
		rotate_buggy(&buggy, 116.0f);
		wait_us(5000);
		move_buggy(&buggy, FORWARD, 1.02f);
		wait_us(5000);
		rotate_buggy(&buggy, 90.0f);
	}
}