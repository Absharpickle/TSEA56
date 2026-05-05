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

// =================================================================
// PARSED PACKET STRUCTS
// =================================================================

// 0x05 Command packet from GUI
typedef struct {
    bool     valid;   // true om paketet är giltigt
    uint8_t  state;   // 0x00–0x03 (körläge)
    uint8_t  target;  // 0x00=wheel, 0x01=arm
    char     action;  // ASCII-kommando ('f','s','e','o','u','v','h', etc.)
} CommandPacket;

// 0x07 Item list packet from GUI
typedef struct {
    bool    valid;
    int     count;                // Antal giltiga varor
    uint8_t items_u[MAX_ITEMS];   // Nod U per vara
    uint8_t items_v[MAX_ITEMS];   // Nod V per vara
} ItemListPacket;

// Styrmodul I2C response
typedef struct {
    bool    valid;
    uint8_t action_done;  // 1 = åtgärden är klar
} StyrResponse;

// Sensor I2C data
typedef struct {
    uint8_t flags;
    uint8_t line_var;
    uint8_t angle;
    uint8_t gyro1;
    uint8_t gyro2;
} SensorData;

// =================================================================
// PARSING: Inkommande paket
// =================================================================

// Parsa ett 0x05-kommandopaket. Returnerar .valid = false om ogiltigt.
CommandPacket parse_command_packet(const unsigned char *buf, int n);

// Parsa ett 0x07-varulistepaket. Validerar kanter mot kartgrafen.
ItemListPacket parse_item_list_packet(const unsigned char *buf, int n);

// Parsa I2C-svar från styrmodul (0x12). Returnerar action_done-flaggan.
StyrResponse parse_styr_response(const unsigned char *buf, int n);

// Parsa sensorpaket (0x10) till struct.
SensorData parse_sensor_packet(const unsigned char *buf);

// =================================================================
// BUILDING: Utgående paket
// =================================================================

// Bygg ett 8-byte motorkommando för I2C till styrmodul.
void build_motor_packet(unsigned char out[PACKET_SIZE],
                        uint8_t state, bool is_pickup,
                        char command,
                        uint8_t line_var, uint8_t gyro1, uint8_t gyro2);

// Bygg ett 14-byte telemetripaket (0x06) för UDP till GUI.
void build_telemetry_packet(unsigned char out[PACKET_SIZE + 6],
                            uint8_t phase, char action, char next_action,
                            uint8_t line_var, uint8_t gyro1, uint8_t gyro2,
                            uint8_t flags, uint8_t node,
                            uint8_t item_idx, uint8_t item_count,
                            char direction, uint8_t action_done);

// Bygg ett variabellängt ruttpaket (0x08) för UDP till GUI.
// Returnerar paketlängden.
int build_route_packet(unsigned char *out, const int *route, int max_nodes);

// =================================================================
// LOGGING
// =================================================================
void log_verification(const unsigned char *sent, char action);
void log_sensor_data(const unsigned char *received);

#endif // PROTOCOL_H
