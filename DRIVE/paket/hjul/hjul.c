#include "hjul.h"
#define F_CPU 16000000UL
#include <util/delay.h>

#define MOTOR2_INVERT

void motor_init(void) {
	DIR_DDR |= (1 << DIR1_PIN) | (1 << DIR2_PIN);
	DDRD    |= (1 << PD5) | (1 << PD4);

	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	TCCR1B = (1 << WGM12)  | (1 << CS11);

	OCR1A = 0;
	OCR1B = 0;
}

void motor_set(uint8_t motor, motor_dir_t dir, uint8_t speed) {
	if (motor == 1) {
		if (dir == FORWARD) DIR_PORT |=  (1 << DIR1_PIN);
		else                DIR_PORT &= ~(1 << DIR1_PIN);
		OCR1A = speed;
		} else {
		if (dir == FORWARD) DIR_PORT &= ~(1 << DIR2_PIN);
		else                DIR_PORT |=  (1 << DIR2_PIN);
		OCR1B = speed;
	}
}

void motor_stop(uint8_t motor) {
	if (motor == 1) OCR1A = 0;
	else            OCR1B = 0;
}

void stop_motor_hard(motor_dir_t dir, brake_mode_t mode)
{
	motor_dir_t not_dir = (dir == FORWARD) ? BACKWARD : FORWARD;

	if (mode == BRAKE_ROTATE) {
		motor_set(1, not_dir, 50);
		motor_set(2, dir,     50);
		} else {
		motor_set(1, dir, 50);
		motor_set(2, dir, 50);
	}

	_delay_ms(10);
	motor_stop(1);
	motor_stop(2);
}

void start_motor_hard(motor_dir_t dir)
{
	motor_set(1, dir,  175);
	motor_set(2, dir, 175);
	_delay_ms(200);
	motor_set(1, dir,  254);
	motor_set(2, dir, 254);
}


//Rotera 
void rotate(motor_dir_t dir)
{
	motor_dir_t not_dir = (dir == FORWARD) ? BACKWARD : FORWARD;
	motor_set(1, dir,  150);
	motor_set(2, not_dir, 150);
	_delay_ms(200);
	motor_set(1, dir,  200);
	motor_set(2, not_dir, 200);
}

void turn(motor_dir_t dir, motor_dir_t bearing)
{
	motor_dir_t not_dir = (dir == FORWARD) ? BACKWARD : FORWARD;
	motor_set(1, bearing,  175-dir*170);
	motor_set(2, bearing, 175-not_dir*170);
	_delay_ms(200);
	motor_set(1, bearing,  254-dir*170);
	motor_set(2, bearing, 254-not_dir*170);
}
