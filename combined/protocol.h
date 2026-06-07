// ------------------------------------------------------
// Markus Hellers, Joel Eberhardsson - 28 maj 2026 - V1.0
// ------------------------------------------------------

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "pathfinding.h"

// --- KOMMUNIKATIONSDEFINITIONER ---
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

// 0x05 Kommandopaket från GUI
typedef struct {
    bool     valid;   // true om paketet är giltigt
    uint8_t  state;   // 0x00–0x03 (körläge)
    uint8_t  target;  // 0x00=wheel, 0x01=arm
    char     action;  // ASCII-kommando eller arm-byte (bits 0-5=joint, 6-7=dir)
} CommandPacket;

// 0x07 Paket med varulista från GUI
typedef struct {
    bool    valid;
    int     count;                // Antal giltiga varor
    uint8_t items_u[MAX_ITEMS];   // Nod U per vara
    uint8_t items_v[MAX_ITEMS];   // Nod V per vara
} ItemListPacket;

// Styrmodulssvar
typedef struct {
    bool    valid;
    uint8_t gas_right;            // Höger gaspådrag
    uint8_t gas_left;             // Vänster gaspådrag
    int8_t  claw_pos_r;           // Klons position i förhållande till basen
    int8_t  claw_pos_z;           // Klons position i z-led
    uint8_t action_done;          // 1 = åtgärden är klar
} StyrResponse;

// Sensorpaket
typedef struct {
    uint8_t flags;                // Statusflaggor
    uint8_t line_var_f;           // Värde från främre linjesensor
    uint8_t line_var_b;           // Värde från bakre linjesensor
    uint8_t angle;                // Vinkel mellan sensorerna
    uint8_t ir;                   // IR-sensor
    uint8_t gyro1;
    uint8_t gyro2;
} SensorData;

// =================================================================
// INKOMMANDE PAKET
// =================================================================

// Parsa ett 0x05-kommandopaket. Returnerar .valid = false om ogiltigt
CommandPacket parse_command_packet(const unsigned char *buf, int n);

// Parsa ett 0x07-varulistepaket. Validerar kanter mot kartgrafen
ItemListPacket parse_item_list_packet(const unsigned char *buf, int n);

// Parsa svar från styrmodul (0x12). Returnerar action_done-flaggan
StyrResponse parse_styr_response(const unsigned char *buf, int n);

// Parsa sensorpaket (0x10)
SensorData parse_sensor_packet(const unsigned char *buf);

// =================================================================
// UTGÅENDE PAKET
// =================================================================

// Bygg ett 8-byte motorkommando till styrmodul
void build_motor_packet(unsigned char out[PACKET_SIZE],
                        uint8_t state, bool is_pickup,
                        char command,
                        uint8_t line_var_f, uint8_t line_var_b, uint8_t gyro1, uint8_t gyro2);

// Bygg ett 18-byte telemetripaket (0x06) till GUI
void build_telemetry_packet(unsigned char out[PACKET_SIZE + 10],
                            uint8_t phase, char action, char next_action,
                            uint8_t line_var_f, uint8_t gyro1, uint8_t gyro2,
                            uint8_t flags, uint8_t node,
                            uint8_t item_idx, uint8_t item_count,
                            char direction, uint8_t action_done,
                            uint8_t gas_right, uint8_t gas_left,
                            int8_t claw_pos_r, int8_t claw_pos_z);

// Bygg ett variabellängt ruttpaket (0x08) till GUI
// Returnerar paketlängden
int build_route_packet(unsigned char *out, const int *route, int max_nodes);

// =================================================================
// LOGGING
// =================================================================
void log_verification(const unsigned char *sent, char action);
void log_sensor_data(const unsigned char *received);

#endif // PROTOCOL_H
