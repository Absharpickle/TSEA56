#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "pathfinding.h"

// --- COMMUNICATION DEFINITIONS ---
#define UDP_PORT 5001
#define BUFFER_SIZE 1024
#define I2C_DEVICE "/dev/i2c-1"
#define STYRKOMM_ADDR 0x12
#define SENSOR_ADDR   0x10
#define PACKET_SIZE 8
#define VERIFY_LOG_FILE "verifikation_keys.txt"

// --- FUNCTIONS ---
void log_verification(const unsigned char *sent, char action);
void log_sensor_data(const unsigned char *received);

#endif // PROTOCOL_H
