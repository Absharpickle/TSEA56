#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include "UART.h"

#define NUM_JOINTS  3
#define BASE_HEIGHT 0
#define STEPS_PER_DEG (1023 / 300)
uint8_t IDjoint8_t[NUM_JOINTS] = {2, 4, 6};
int16_t rawstep;
int16_t offset[NUM_JOINTS] = {362, 464, -522};
int16_t angles[NUM_JOINTS] = {0, 0, 0};
int16_t maxangle[8] = { 0x33a, 0x32b, 0x335, 0x38e, 0x334, 0x359, 0x200, 0x205 };
int16_t minangle[8] = { 0xCF, 0xC9, 0xcb, 0xc9, 0x75, 0xd5, 0x200, 0x48 };
int16_t min_z = -170;

void init_UART(void)
{
	
	uint16_t ubrr = 0;
	UBRR0H = (ubrr >> 8);
	UBRR0L = ubrr;
	UCSR0B = (1 << RXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	uint8_t lock[] = { 0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x2F, 0x01, 0xDD };
	uint8_t write_torque_all[] = { 0xFF, 0xFF, 0xFE, 0x1C, 0x83, 0x22, 0x02,
		0x01, 0xFF, 0x01,
		0x02, 0xFF, 0x01,
		0x03, 0xFF, 0x01,
		0x04, 0xFF, 0x01,
		0x05, 0xFF, 0x01,
		0x06, 0xFF, 0x01,
		0x07, 0xFF, 0x01,
		0x08, 0xFF, 0x01,
		0xFF
	};
	uint8_t write_speed_all[] = { 0xFF, 0xFF, 0xFE, 0x1C, 0x83, 0x20, 0x02,
		0x01, 0x40, 0x00,
		0x02, 0x40, 0x00,
		0x03, 0x40, 0x00,
		0x04, 0x80, 0x00,
		0x05, 0x80, 0x00,
		0x06, 0x40, 0x00,
		0x07, 0x40, 0x00,
		0x08, 0x80, 0x00,
	0xFF};
	uint8_t write_compliance[] = { 0xFF, 0xFF, 0xFE, 0x1C, 0x83, 0x1C, 0x02,
		0x01, 0x60, 0x60,
		0x02, 0x60, 0x60,
		0x03, 0x60, 0x60,
		0x04, 0x60, 0x60,
		0x05, 0x60, 0x60,
		0x06, 0x60, 0x60,
		0x07, 0x60, 0x60,
		0x08, 0x60, 0x60,
	0xFF };
	send_UART(lock, sizeof(lock));
	send_UART(write_torque_all, sizeof(write_torque_all));
	send_UART(write_speed_all, sizeof(write_speed_all));
	send_UART(write_compliance, sizeof(write_compliance));
	_delay_ms(10);
	
	
	
}

uint8_t uart0_receive(void)
{
	uint16_t timeout = 0xF;
	while (!(UCSR0A & (1 << RXC0)))
	{
		timeout--;
	}
	if (timeout == 0)
	{
		return 0;
	} else
	{
		return UDR0;
	}
}

void uart0_receive_buf(uint8_t *buf, uint8_t len, uint8_t start)
{
	for (uint8_t i = start; i < len; i++) {
		buf[i] = uart0_receive();
	}
}

void receive_UART(uint8_t *buf)
{
	uart0_receive_buf(buf, 4, 0);
	uart0_receive_buf(buf, buf[3] + 4, 4);
}

void send_UART(uint8_t *buf, uint8_t len)
{
	UCSR0B = (1 << TXEN0);
	uint8_t checksum = 0xff;
	for (uint8_t i = 0; i < len-1; i++) {
		while (!(UCSR0A & (1 << UDRE0)));
		UDR0 = buf[i];
		if(i>=2) checksum -= buf[i];
	}
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = checksum;

	while (!(UCSR0A & (1 << TXC0)));
	UCSR0A |= (1 << TXC0);
	
	UCSR0B = (1 << RXEN0);
	_delay_us(10);
}

void set_mirror_position(uint8_t id_low, uint16_t pos1)
{
	if (id_low == 2 || id_low == 4)
	{
		uint16_t pos2 = 1023 - pos1;
		uint8_t pkt[14] = {
			0xFF, 0xFF, 0xFE, 0x0A, 0x83, 0x1E, 0x02,
			id_low, (pos1 & 0xFF), (pos1 >> 8),
			id_low+1, (pos2 & 0xFF), (pos2 >> 8),
			0x00
		};
		send_UART(pkt, sizeof(pkt));
	}
}

void set_position(uint8_t id, uint16_t pos)
{
	if (!(id == 2 || id == 4 || id == 3 || id == 5))
	{
		uint8_t pkt[] = {0xFF, 0xff, id, 0x05, 0x03, 0x1E,
		(pos & 0xFF), (pos >> 8), 0xFF};
		send_UART(pkt, sizeof(pkt));
	}
}

uint16_t read_position(uint8_t motor_id)
{
	uint8_t pkt[] = { 0xFF, 0xFF, motor_id, 0x04, 0x02, 0x24, 0x02, 0xFF };
	uint8_t svar[8];

	send_UART(pkt, sizeof(pkt));
	receive_UART(svar);

	return svar[5] | ((uint16_t)svar[6] << 8);

}

void read_adress(uint8_t id, uint8_t adress, uint8_t len)
{
	uint8_t pkt[] = { 0xFF, 0xFF, id, 0x04, 0x02, adress, len, 0xFF };
	uint8_t svar[6+len];

	send_UART(pkt, sizeof(pkt));
	receive_UART(svar);
}


void torque_on_motor(uint8_t motor_id)
{
	uint8_t pkt[] = { 0xFF, 0xFF, motor_id, 0x04, 0x03, 0x18, 0x01, 0xDD };
	send_UART(pkt, sizeof(pkt));
}

void torque_off_motor(uint8_t motor_id)
{
	uint8_t pkt[] = { 0xFF, 0xFF, motor_id, 0x04, 0x03, 0x18, 0x00, 0xDE };
	send_UART(pkt, sizeof(pkt));
}

void torque_limit_zero(uint8_t motor_id)
{
	uint8_t pkt[] = { 0xFF, 0xFF, motor_id, 0x05, 0x03, 0x22, 0x00, 0x00, 0xDD };
	send_UART(pkt, sizeof(pkt));
	
};

void lys(uint8_t motor_id)
{
	uint8_t pkt[] = { 0xFF, 0xFF, motor_id, 0x04, 0x03, 0x19, 1, 0xF2 };
	send_UART(pkt, sizeof(pkt));
}

void ping(uint8_t motor_id)
{
	uint8_t pkt[] = { 0xFF, 0xFF, motor_id, 0x02, 0x01, 0xFA }; // FF, FF, ID,  Lngd, inst, par,checksum
	send_UART(pkt, sizeof(pkt));
}

void start_position()
{
	uint8_t command[] = { 0xFF, 0xFF, 0xFE, 0x0D, 0x83, 0x1E, 0x02,
		0x01, 0xFF, 0x01,
		0x06, 0x3D, 0x03,
		0x07, 0x00, 0x02,
		0xFF
	};
	send_UART(command, sizeof(command));
	_delay_ms(500);
	set_mirror_position(0x02, 0xC9);
	set_mirror_position(0x04, 0xC9);
}

void attack_position(pick_side side)
{
	uint8_t low_byte = 0xC9 - side*0x94;
	uint8_t high_byte = 0x00 + side*3;

	uint8_t command[] = { 0xFF, 0xFF, 0xFE, 0x0D, 0x83, 0x1E, 0x02,
		0x01, low_byte, high_byte,
		0x06, 0xDF, 0x01,
		0x07, 0x07, 0x02,
		0xFF
	};
	set_mirror_position(0x02, 0x1E1);
	set_mirror_position(0x04, 0x31F);
	_delay_ms(500);
	send_UART(command, sizeof(command));
}

void pick_position()
{
	uint8_t command[] = { 0xFF, 0xFF, 0xFE, 0x0A, 0x83, 0x1E, 0x02,
		0x06, 0x94, 0x01,
		0x07, 0x07, 0x02,
		0xFF
	};
	set_mirror_position(0x02, 0x269);
	set_mirror_position(0x04, 0x1fb);
	_delay_ms(500);
	send_UART(command, sizeof(command));
}

void close_claw()
{
	set_position(8, 0x48);
}

void open_claw()
{
	set_position(8, 0x205);
}

void pick_item(pick_side side)
{
	start_position();
	_delay_ms(100);
	attack_position(side);
	open_claw();
	_delay_ms(3500);
	pick_position();
	_delay_ms(1000);
	close_claw();
	_delay_ms(2500);
	attack_position(side);
	_delay_ms(1500);
	start_position();
}

float step_to_rad(uint16_t raw_step, uint16_t off) {
	float rad;
	rad = ((float)(raw_step + off) / 1023.0f) * (300.0f * M_PI / 180.0f);
	if (rad > 2*M_PI) {
		rad = rad - 2*M_PI;
	}
	return rad;
}


cumResult cumAngle(void) {
	float rad_angles[3];
	
	//Ls vinkel
	for (uint8_t i = 0; i < NUM_JOINTS; i++) {
		rawstep = read_position(IDjoint8_t[i]);
		angles[i] = rawstep;
		rad_angles[i] = step_to_rad(angles[i], offset[i]);
	}
	
	float a1 = rad_angles[0];
	float a2 = rad_angles[0] + rad_angles[1];
	float a3 = rad_angles[0] + rad_angles[1] + rad_angles[2];
	
	cumResult load = {a1, a2, a3};

	return load;
}

cumResult cumAngle2(uint16_t ang1, uint16_t ang2, uint16_t ang3)
{
	uint16_t angles[NUM_JOINTS] = {ang1, ang2, ang3};
	float rad_angles[NUM_JOINTS];
	//Ls vinkel
	for (uint8_t i = 0; i < NUM_JOINTS; i++) {
		angles[i] = rawstep;
		rad_angles[i] = step_to_rad(angles[i], offset[i]);
	}
	
	float a1 = rad_angles[0];
	float a2 = rad_angles[0] + rad_angles[1];
	float a3 = rad_angles[0] + rad_angles[1] + rad_angles[2];
	
	cumResult load = {a1, a2, a3};

	return load;
}

FK2DResult forward_kinematics_2d(void) {
	
	cumResult load = cumAngle();

	int16_t r = LIMB_LENGTHS[0] * cosf(load.a1)
	+ LIMB_LENGTHS[1] * cosf(load.a2)
	+ LIMB_LENGTHS[2] * cosf(load.a3);

	int16_t z = BASE_HEIGHT
	+ LIMB_LENGTHS[0] * sinf(load.a1)
	+ LIMB_LENGTHS[1] * sinf(load.a2)
	+ LIMB_LENGTHS[2] * sinf(load.a3);

	FK2DResult result = {z, r};
	return result;
}

FK2DResult test_forward_kinematics_2d(uint16_t ang1, uint16_t ang2, uint16_t ang3)
{
	
	cumResult load = cumAngle2(ang1, ang2, ang3);

	int16_t r = LIMB_LENGTHS[0] * cosf(load.a1)
	+ LIMB_LENGTHS[1] * cosf(load.a2)
	+ LIMB_LENGTHS[2] * cosf(load.a3);

	int16_t z = BASE_HEIGHT
	+ LIMB_LENGTHS[0] * sinf(load.a1)
	+ LIMB_LENGTHS[1] * sinf(load.a2)
	+ LIMB_LENGTHS[2] * sinf(load.a3);

	FK2DResult result = {z, r};
	return result;
}

void compute_jacobian(int16_t J[2][3]) {
	cumResult load = cumAngle();

	J[0][0] = -LIMB_LENGTHS[0]*sinf(load.a1) - LIMB_LENGTHS[1]*sinf(load.a2) - LIMB_LENGTHS[2]*sinf(load.a3);
	J[0][1] =                                 - LIMB_LENGTHS[1]*sinf(load.a2) - LIMB_LENGTHS[2]*sinf(load.a3);
	J[0][2] =                                                                  - LIMB_LENGTHS[2]*sinf(load.a3);

	J[1][0] = LIMB_LENGTHS[0]*cosf(load.a1) + LIMB_LENGTHS[1]*cosf(load.a2) + LIMB_LENGTHS[2]*cosf(load.a3);
	J[1][1] =                                  LIMB_LENGTHS[1]*cosf(load.a2) + LIMB_LENGTHS[2]*cosf(load.a3);
	J[1][2] =                                                                   LIMB_LENGTHS[2]*cosf(load.a3);
}

IKResult inverse_kinematics(pos_change change_dir, coord_to_change change_coord, int16_t min_z, float tol, uint8_t max_iter, float alpha)
{
	uint8_t change_magnitude = 10;
	FK2DResult starting_position = forward_kinematics_2d();
	FK2DResult goal = {starting_position.z + (change_dir * !change_coord * change_magnitude), starting_position.r + (change_dir * change_coord * change_magnitude)};
	if (goal.z < min_z){goal.z = min_z;}
	
	uint16_t iter;
	for (iter = 0; iter < max_iter; iter++) {
		// Ta reda p position
		FK2DResult current_position = forward_kinematics_2d();
		FK2DResult err;

		// Ta reda p felet
		err.r = goal.r - current_position.r;
		err.z = goal.z - current_position.z;

		// Ifall felet under en grns avsluta
		float dist = sqrtf((float)err.r * err.r + (float)err.z * err.z);
		if (dist <= tol) {
			break;
		}

		// Skapa jacobian (gradient descent, riktning i vinklar som minskar felet snabbast)
		int16_t J[2][3];
		compute_jacobian(J);

		// Ger oss dtta som vi ska ndra nuvarande vinkel
		for (uint8_t i = 0; i < NUM_JOINTS; i++) {
			float dtheta = alpha * (J[0][i] * (float)err.r + J[1][i] * (float)err.z);
			int16_t dstep = (int16_t)(dtheta * RAD_TO_STEP);
			angles[i] = (uint16_t)clip((int16_t)angles[i] + dstep, i);
		}
	}
	
	IKResult res;
	res.angle1 = angles[0];
	res.angle2 = angles[1];
	res.angle3 = angles[2];
	return res;
}
void move_joint(uint8_t motor_id, pos_change change)
{
	uint8_t magnitude = 40;
	uint16_t current_pos;
	uint16_t goal_pos;
	
	current_pos = read_position(motor_id);
	goal_pos = current_pos + change*magnitude;
	if (goal_pos > maxangle[motor_id-1])
	{
		goal_pos = maxangle[motor_id-1];
	} else if (goal_pos < minangle[motor_id-1])
	{
		goal_pos = minangle[motor_id-1];
	}
	if (motor_id == 2||motor_id == 4)
	{
		set_mirror_position(motor_id, goal_pos);
	} else if (motor_id != 3||motor_id !=5)
	{
		set_position(motor_id, goal_pos);
	}
	_delay_ms(100);
	
}
