// ------------------------------------------------------
// Markus Hellers, Joel Eberhardsson - 28 maj 2026 - V1.0
// ------------------------------------------------------

#include "protocol.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

// =================================================================
// INKOMMANDE PAKET
// =================================================================

// Parsa 0x05-kommandopaket
CommandPacket parse_command_packet(const unsigned char *buf, int n) {
    CommandPacket pkt = { .valid = false };
    if (n == PACKET_SIZE && buf[0] == 0x05) {
        pkt.valid  = true;
        pkt.state  = buf[1];  
        pkt.target = buf[2];
        pkt.action = (char)buf[3];
    }
    return pkt;
}

// Parsa varupaket
ItemListPacket parse_item_list_packet(const unsigned char *buf, int n) {
    ItemListPacket pkt = { .valid = false, .count = 0 };
    if (n < 4 || buf[0] != 0x07) return pkt;

    int num = buf[1];
    int expected_len = 3 + 2 * num;
    if (num <= 0 || num > MAX_ITEMS || n != expected_len || buf[n - 1] != 0xFF) {
        printf("[!] Ogiltigt paket (n=%d, count=%d)\n", n, num);
        return pkt;
    }

    pkt.valid = true;
    for (int i = 0; i < num; i++) {
        uint8_t iu = buf[2 + 2 * i];
        uint8_t iv = buf[3 + 2 * i];
        if (iu < NODES && iv < NODES && vag[iu][iv]) {
            pkt.items_u[pkt.count] = iu;
            pkt.items_v[pkt.count] = iv;
            pkt.count++;
        } else {
            printf("[!] Hoppar över ogiltig vara %d, %d\n", iu, iv);
        }
    }
    return pkt;
}

// Parsa svar från styrmodul
StyrResponse parse_styr_response(const unsigned char *buf, int n) {
    StyrResponse resp = { .valid = false, .action_done = 0 };
    if (n == PACKET_SIZE) {
        resp.valid       = true;
        resp.gas_right   = buf[0];  // Höger gaspådrag
        resp.gas_left    = buf[1];  // Vänster gaspådrag
        resp.claw_pos_r  = buf[2];  // Klons position i r-led
        resp.claw_pos_z  = buf[3];  // Klons position i z-led
        resp.action_done = buf[4];  // Action_done flagga
    }
    return resp;
}

// Parsa sensorpaket
SensorData parse_sensor_packet(const unsigned char *buf) {
    SensorData s;
    s.flags    = buf[0];        // Flaggor
    s.line_var_f = buf[1];      // Linjevar fram
    s.line_var_b = buf[2];      // Linjevar bak
    s.angle    = buf[3];        // Vinkel mellan sensorerna
    s.ir       = buf[5];        // IR
    s.gyro1    = buf[6];        // Gyro 1
    s.gyro2    = buf[7];        // Gyro 2
    return s;
}

// =================================================================
// UTGÅENDE PAKET
// =================================================================

// Bygg ett motorpaket
void build_motor_packet(unsigned char out[PACKET_SIZE],
                        uint8_t state, bool is_pickup,
                        char command,
                        uint8_t line_var_f, uint8_t line_var_b, uint8_t gyro1, uint8_t gyro2) {
    out[0] = 0x05;                      // Paket-ID
    out[1] = state;                     // State
    out[2] = is_pickup ? 0x01 : 0x00;   // Target
    out[3] = (unsigned char)command;    // Action
    out[4] = line_var_f;                // Line var fram
    out[5] = line_var_b;                // Line var bak
    out[6] = gyro1;                     // Gyro 1
    out[7] = gyro2;                     // Gyro 2
   
}

// Bygg ett telemetripaket
void build_telemetry_packet(unsigned char out[PACKET_SIZE + 10],
                            uint8_t phase, char action, char next_action,
                            uint8_t line_var_f, uint8_t gyro1, uint8_t gyro2,
                            uint8_t flags, uint8_t node,
                            uint8_t item_idx, uint8_t item_count,
                            char direction, uint8_t action_done,
                            uint8_t gas_right, uint8_t gas_left,
                            int8_t claw_pos_r, int8_t claw_pos_z) {
    out[0]  = 0x06;                         // Paket-ID
    out[1]  = phase;                        // Tillståndsvariabel 
    out[2]  = (unsigned char)action;        // Nuvarande action
    out[3]  = (unsigned char)next_action;   // Nästa action
    out[4]  = line_var_f;                   // Line var fram
    out[5]  = gyro1;                        // Gyro 1
    out[6]  = gyro2;                        // Gyro 2
    out[7]  = flags;                        // Flaggor
    out[8]  = node;                         // Aktuell nod
    out[9]  = item_idx;                     // Aktuell index
    out[10] = item_count;                   // Total antal varor
    out[11] = (unsigned char)direction;     // Körriktning
    out[12] = action_done;                  // Status
    out[13] = gas_right;                    // Höger gaspådrag
    out[14] = gas_left;                     // Vänster gaspådrag
    out[15] = (unsigned char)claw_pos_r;    // Klons position i r
    out[16] = (unsigned char)claw_pos_z;    // Klons position i z
    out[17] = 0xFF;                         // Avslut
}

// Bygg ett ruttpaket (0x08) till GUI
int build_route_packet(unsigned char *out, const int *route, int max_nodes) {
    int count = 0;
    while (count < max_nodes && route[count] != STOP) count++;

    out[0] = 0x08;                              // Paket-ID
    out[1] = (unsigned char)count;              // Antal noder
    for (int i = 0; i < count; i++) {
        out[2 + i] = (unsigned char)route[i];   // Nodnummer
    }
    out[2 + count] = 0xFF;                      // Avslut
    return 3 + count;                           // Variabel paketlängd
}

// =================================================================
// LOGGING
// =================================================================

// Logga skickade kommandon till verifikationsfil
void log_verification(const unsigned char *sent, char action) {
    FILE *f = fopen(VERIFY_LOG_FILE, "a");
    if (f == NULL) return;
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    fprintf(f, "[%02d:%02d:%02d] ACTION '%c'\nSKICKAT (0x12): ", t->tm_hour, t->tm_min, t->tm_sec, action);
    for (int i = 0; i < PACKET_SIZE; i++) fprintf(f, "%02X ", sent[i]);
    fprintf(f, "\n\n");
    fclose(f);
}

// Logga inkommande sensordata till verifikationsfil
void log_sensor_data(const unsigned char *received) {
    FILE *f = fopen(VERIFY_LOG_FILE, "a");
    if (f == NULL) return;
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    fprintf(f, "[%02d:%02d:%02d] SENSOR LÄST (0x10): ", t->tm_hour, t->tm_min, t->tm_sec);
    for (int i = 0; i < PACKET_SIZE; i++) fprintf(f, "%02X ", received[i]);
    fprintf(f, "\n\n");
    fclose(f);
}
