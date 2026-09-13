#ifndef HJUL_H
#define HJUL_H

#include <avr/io.h>
#include <stdint.h>

// DIR-pinnar (Port A)
#define DIR_DDR     DDRA
#define DIR_PORT    PORTA
#define DIR1_PIN    PA6   // DIR_1
#define DIR2_PIN    PA7   // DIR_2

typedef enum { FORWARD = 1, CW = 1, RIGHT_TURN = 1, BACKWARD = 0, CCW = 0, LEFT_TURN = 0 } motor_dir_t;
typedef enum {
	BRAKE_NORMAL,
	BRAKE_ROTATE
} brake_mode_t;
	
void motor_init(void);
void motor_set(uint8_t motor, motor_dir_t dir, uint8_t speed); // 1 = höger, 2 = vänster
void motor_stop(uint8_t motor);
void stop_motor_hard(motor_dir_t dir, brake_mode_t mode);
void start_motor_hard(motor_dir_t dir);
void rotate(motor_dir_t dir);
void turn(motor_dir_t dir, motor_dir_t bearing);

#endif