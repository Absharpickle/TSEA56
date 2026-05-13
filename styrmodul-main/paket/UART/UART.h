#ifndef UART_H
#define UART_H

#include <stdint.h>

#define NUM_JOINTS  3
#define BASE_HEIGHT 0
#define STEPS_PER_DEG (1023 / 300)
#define RAD_TO_STEP (1023.0f / (300.0f * M_PI / 180.0f))

//Inverskinematik
static const uint16_t LIMB_LENGTHS[NUM_JOINTS] = {150, 145, 150};
static const float tol = 2;
static const uint16_t max_iter = 10;
static const uint8_t alpha = 25;

uint8_t IDjoint8_t[NUM_JOINTS];
int16_t rawstep;
int16_t offset[NUM_JOINTS];
int16_t angles[NUM_JOINTS];
int16_t maxangle[8];
int16_t minangle[8];
int16_t min_z;
typedef enum {RIGHT_PICK = 0, LEFT_PICK = 1} pick_side;
typedef enum {INC = 1, DEC = -1} pos_change;
typedef enum {Z_Coord = 0, R_Coord = 1} coord_to_change;
typedef enum{STOP_ARM = 0, INC_ARM = 0b10000000, DEC_ARM = 0b01000000} arm_action_t;

typedef struct {
	uint16_t angle1;
	uint16_t angle2;
	uint16_t angle3;
} IKResult;

typedef struct {
	int16_t z;
	int16_t r;
} FK2DResult;

typedef struct {
	float a1;
	float a2;
	float a3;
} cumResult;

typedef struct {
	int16_t z;
	int16_t r;
} Coordinate;

void init_UART(void);
void receive_UART(uint8_t *buf);
void send_UART(uint8_t *buf, uint8_t len);
void set_mirror_position(uint8_t id_low, uint16_t pos1);
void set_position(uint8_t id, uint16_t pos);
uint16_t read_position(uint8_t motor_id);
void read_adress(uint8_t id, uint8_t adress, uint8_t len);
void torque_on_motor(uint8_t motor_id);
void torque_off_motor(uint8_t motor_id);
void torque_limit_zero(uint8_t motor_id);
void lys(uint8_t motor_id);
void ping(uint8_t motor_id);
void start_position();
void attack_position(pick_side side);
void pick_position();
void close_claw();
void open_claw();
void pick_item(pick_side side);
FK2DResult forward_kinematics_2d(void);
IKResult inverse_kinematics(pos_change change_dir, coord_to_change change_coord, int16_t min_z, float tol, uint8_t max_iter, float alpha);
FK2DResult test_forward_kinematics_2d(uint16_t ang1, uint16_t ang2, uint16_t ang3);
uint16_t clip(uint16_t angle, uint8_t motor_id);
void move_joint(uint8_t motor_id, pos_change change);

#endif