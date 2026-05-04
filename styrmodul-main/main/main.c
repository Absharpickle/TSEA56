#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <stdbool.h>
#include "UART.h"
#include "hjul.h"
#define SLAVE_ADDRESS 0x12

volatile bool drive_cmd_updated_soon = false;
volatile bool arm_cmd_updated_soon = false;
volatile bool drive_cmd_updated = false;
volatile bool arm_cmd_updated = false;
volatile float accumulated_dbg = 0;
volatile int8_t gyro1_dbg = 0;
volatile int8_t gyro2_dbg = 0;
volatile uint8_t drive_cmd = 0;
volatile uint8_t arm_cmd = 0;
volatile int8_t line_error = 0;
volatile uint8_t rx_buffer[8];
volatile uint8_t tx_buffer[5] = {0x00}; 
volatile uint8_t rx_index = 0;
volatile uint8_t tx_index = 0;          // Index f?r s?ndning
volatile uint8_t nytt_paket_mottaget = 0;
volatile uint8_t action_done = 0;

uint8_t manual_drive_mode = 0;
uint8_t manual_arm_mode = 0;
pos_change inc_or_dec = INC;
uint8_t current_drive_cmd = 0;
uint8_t current_arm_cmd = 0;
uint8_t motor_id = 1;
volatile uint8_t new_command = 0;
uint8_t current_man_arm_joint = 1;
motor_dir_t current_dir = FORWARD;
brake_mode_t current_brake_mode = BRAKE_NORMAL;
arm_action_t current_man_arm_act = STOP_ARM;

enum command_val
{
	CMD_FORWARD = 0x66,	//f
	CMD_BACKWARD = 0x62,	//b
	CMD_STOP = 0x73,	//s
	CMD_RIGHT = 0x72, //r
	CMD_LEFT = 0x6c, //l
	CMD_CCW = 0x6f, //o
	CMD_CW = 0x65, //e
	CMD_PLOCKA_LEFT = 0x68, //v
	CMD_PLOCKA_RIGHT = 0x76, //h
	CMD_TURN_180 = 0x75 //g
};

void TWI_init_slave(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
	TWAR = (SLAVE_ADDRESS << 1);
	TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
//	TCCR1A = 0;
// 	TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10);
// 	OCR1A  = 3905;
// 	TIMSK1 |= (1 << OCIE1A);
}



ISR(TWI_vect)
{
	cli();
	tx_buffer[0] =	0; //PWM r
	tx_buffer[1] =	0; //PWM_l
	tx_buffer[2] =	0; //Gripklo r
	tx_buffer[3] =	0; //Gripklo z
	tx_buffer[4] =	action_done; //Action done flagga
	uint8_t status = TWSR & 0xF8;
	switch (status)
	{
		case 0x60: //master vill skriva p? styrmodul
		rx_index = 0;
		TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE);
		break;

		case 0x80: //databyte kommer
		{
			if (rx_index < 8) {rx_buffer[rx_index] = TWDR;}
			if (rx_index == 2)
			{
				switch (rx_buffer[rx_index])
				{
					case 0:
					drive_cmd_updated_soon = true;
					break;
					case 1:
					arm_cmd_updated_soon = true;
					break;
				}
			}
			rx_index++;
			gyro1_dbg = rx_buffer[5];
			gyro2_dbg = rx_buffer[6];
			break;
		}

		case 0xA8: //master vill l?sa fr?n styrmodul
		tx_index = 0;
		TWDR = tx_buffer[tx_index++];
		break;

		case 0xB8: //skicka data till master
		TWDR = (tx_index < 5) ? tx_buffer[tx_index++] : 0x00;
		if (tx_index == 5 && tx_buffer[4] == 1)
		{
			action_done = 0;
		}
		
		
		break;

		case 0xC0: //sista byte skickad

		case 0xC8: //sista byte skickad men master vill ha mer (felfall)

		case 0xA0: //stoppbyte
		if (drive_cmd_updated_soon)
		{
			drive_cmd_updated = true;
			drive_cmd_updated_soon = false;
		}
		if (arm_cmd_updated_soon)
		{
			arm_cmd_updated = true;
			arm_cmd_updated_soon = false;
		}
		break;
	}
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
	sei();
}

void drive_straight(motor_dir_t direction)
{
	start_motor_hard(direction);
	while (new_command == 0)
	{
		if (current_drive_cmd != rx_buffer[3])
		{
			new_command = 1;
		}
	}
}

void follow_line_pid(motor_dir_t direction)
{
	int u = 0;
	int8_t p_error = 0;
	int prev_error = 0;
	int i_error = 0;
	int d_error = 0;
	int k_p = 6; //100
	int k_i = 0;
	int k_d = 6; //3
	int16_t PWM_r;
	int16_t PWM_l;
	int16_t PWM_max = 150; //175
	int16_t PWM_min = 0; //25
	
	while (new_command == 0)
	{
		//PORTB |= (1 << PB0);
		if (current_drive_cmd != rx_buffer[3])
		{
			new_command = 1;
		}
		
		p_error = (int8_t)rx_buffer[4];
		i_error += p_error;
		d_error = p_error - prev_error;
		
		u = (k_p*p_error + k_i*(i_error/100) + k_d*d_error);
		
		PWM_r = (PWM_max - 75 + u);
		if (PWM_r > PWM_max) {PWM_r = PWM_max;}
		if (PWM_r < PWM_min) {PWM_r = PWM_min;}
		
		PWM_l = (PWM_max - 75 - u);
		if (PWM_l > PWM_max) {PWM_l = PWM_max;}
		if (PWM_l < PWM_min) {PWM_l = PWM_min;}
		
		motor_set(1, direction, PWM_r);
		motor_set(2, direction, PWM_l);
		prev_error = p_error;
		
		_delay_ms(2);
		//PORTB &= ~(1 << PB0);
	}
}

void rotate_pid(motor_dir_t spin_dir, uint8_t degrees)
{
	int32_t goal = (degrees == 0) ? 50 : 93; //10375, 19000
	motor_dir_t not_spin_dir = (spin_dir == FORWARD) ? BACKWARD : FORWARD;
	
	action_done = 0;
	int32_t k_p = 15, k_d = 0;
	float accumulated = 0;
	int32_t prev_error  = goal;
	int16_t PWM_max = 175 - degrees*50;
	int16_t PWM_min = 20;
	
	while (new_command == 0)
	{
		if (current_drive_cmd != rx_buffer[3])
		{
			new_command = 1;
		}
		int16_t gyro = (int16_t)rx_buffer[5] + ((int16_t)rx_buffer[6] << 8); //prelimin�rt skrivet som little endian
		gyro1_dbg = rx_buffer[5];
		gyro2_dbg = rx_buffer[6];
		if (spin_dir == CCW)
		{
			accumulated += (float)gyro/100;
		} else if (spin_dir == CW) {
			accumulated -= (float)gyro/100;
		}
		accumulated_dbg = accumulated;

		int32_t error   = goal - accumulated;
		if (error > 5 || error < -5)  // utom 3 grader
		{
			int32_t d_error = (error - prev_error) * 100;  // de/dt, skalat f?r 10ms

			int32_t u = (int32_t)k_p * error + (int32_t)k_d * d_error;

			
			int16_t pwm = (int16_t)(u >= 0) ? u : -u;
			if (pwm > PWM_max) pwm = PWM_max;
			if (pwm < PWM_min) pwm = PWM_min;
			motor_set(1, spin_dir, (uint8_t)pwm);
			motor_set(2, not_spin_dir, (uint8_t)pwm);

		} else
		{
			motor_set(1, spin_dir, 0);
			motor_set(2, not_spin_dir, 0);
			//accumulated = 0;
			action_done = 1;
			_delay_ms(2000);
		}
		prev_error = error;

		

		_delay_ms(10);

	}
}

int main()
{
	
	init_UART();
	motor_init();
	TWI_init_slave();
	DDRB |= (1 << PB0);
	start_position();
	_delay_ms(1000);
	sei();

	while(1)
	{

		if (drive_cmd_updated == true) {
			
			cli();
			new_command = 0;
			manual_drive_mode = (2 & rx_buffer[1]);
			current_drive_cmd = rx_buffer[3];
			drive_cmd_updated = false;

			sei();
			if (manual_drive_mode == 2)
			{
				
				switch(current_drive_cmd)
				{
					case CMD_FORWARD:
					current_dir = FORWARD;
					current_brake_mode = BRAKE_NORMAL;
					drive_straight(current_dir);
					break;
					case CMD_BACKWARD:
					current_dir = BACKWARD;
					current_brake_mode = BRAKE_NORMAL;
					drive_straight(current_dir);
					break;
					case CMD_CCW:
					current_dir = CCW;
					current_brake_mode = BRAKE_ROTATE;
					rotate(current_dir);
					break;
					case CMD_CW:
					current_dir = CW;
					current_brake_mode = BRAKE_ROTATE;
					rotate(current_dir);
					break;
					case CMD_LEFT:
					current_brake_mode = BRAKE_NORMAL;
					turn(LEFT_TURN, current_dir);
					break;
					case CMD_RIGHT:
					current_brake_mode = BRAKE_NORMAL;
					turn(RIGHT_TURN, current_dir);
					break;
					case CMD_STOP:
					stop_motor_hard(current_dir, current_brake_mode);
					action_done = 1;
					while (new_command == 0)
					{
						if ((current_drive_cmd != rx_buffer[3])||(arm_cmd_updated == true))
						{
							new_command = 1;
						}
					}
					break;
				}
			}
			else if (manual_drive_mode == 0)
			{
				switch(current_drive_cmd)
				{
					case CMD_FORWARD:
					current_dir = FORWARD;
					current_brake_mode = BRAKE_NORMAL;
					//start_motor_hard(current_dir);
					follow_line_pid(current_dir);
					break;
					case CMD_BACKWARD:
					current_dir = BACKWARD;
					current_brake_mode = BRAKE_NORMAL;
					start_motor_hard(current_dir);
					follow_line_pid(current_dir);
					break;
					case CMD_CCW:
					motor_set(1, BACKWARD, 100);
					motor_set(2, BACKWARD, 100);
					_delay_ms(50);
					motor_set(1, BACKWARD, 0);
					motor_set(2, BACKWARD, 0);
					current_dir = CCW;
					current_brake_mode = BRAKE_ROTATE;
					rotate_pid(current_dir, 0);
					break;
					case CMD_CW:
					motor_set(1, BACKWARD, 100);
					motor_set(2, BACKWARD, 100);
					_delay_ms(50);
					motor_set(1, BACKWARD, 0);
					motor_set(2, BACKWARD, 0);
					current_dir = CW;
					current_brake_mode = BRAKE_ROTATE;
					rotate_pid(current_dir, 0);
					break;
					case CMD_TURN_180:
					motor_set(1, BACKWARD, 0);
					motor_set(2, BACKWARD, 0);
					current_dir = CCW;
					current_brake_mode = BRAKE_ROTATE;
					rotate_pid(current_dir, 1);
					break;
					case CMD_STOP:
					if (current_brake_mode == BRAKE_NORMAL) {
						motor_set(1, current_dir, 50);
						motor_set(2, current_dir, 50);
					}
					_delay_ms(250);
					stop_motor_hard(current_dir, current_brake_mode);
					action_done = 1;
					while (new_command == 0)
					{
						if ((current_drive_cmd != rx_buffer[3])||arm_cmd_updated == true)
						{
							new_command = 1;
						}
					}
					break;
				}
			}
		}
		if (arm_cmd_updated == true)
		{
			cli();
			new_command = 0;

			manual_arm_mode = (1 & rx_buffer[1]);
			arm_cmd_updated = false;
			current_arm_cmd = rx_buffer[3];
			sei();
			
			if (manual_arm_mode == 1)
			{
				current_man_arm_act = current_arm_cmd & 0b11000000;
				switch (current_arm_cmd & 0b00111111)
				{
					case 0b00000001:
					motor_id = 1;
					break;
					case 0b00000010:
					motor_id = 2;
					break;
					case 0b00000100:
					motor_id = 4;
					break;
					case 0b00001000:
					motor_id = 6;
					break;
					case 0b00010000:
					motor_id = 7;
					break;
					case 0b00100000:
					motor_id = 8;
					break;
				}
				if (current_man_arm_act == INC_ARM)
				{
					inc_or_dec = INC;
					} else if (current_man_arm_act == DEC_ARM){
					inc_or_dec = DEC;
				}
				while (new_command == 0)
				{
					if ((current_arm_cmd != rx_buffer[3])||drive_cmd_updated == true)
					{
						new_command = 1;
					}
					move_joint(motor_id, inc_or_dec);
				}
				} else {
				switch (current_arm_cmd)
				{
					case CMD_PLOCKA_LEFT:
					pick_item(LEFT_PICK);
					_delay_ms(1000);
					action_done = 1;
					break;
					case CMD_PLOCKA_RIGHT:
					pick_item(RIGHT_PICK);
					_delay_ms(1000);
					action_done = 1;
					break;
				}
			}
		}
	}
	return 0;
}
