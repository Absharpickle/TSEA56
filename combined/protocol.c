#include "protocol.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

// =================================================================
// PARSING: Inkommande paket
// =================================================================

CommandPacket parse_command_packet(const unsigned char *buf, int n) {
    CommandPacket pkt = { .valid = false };
    if (n == PACKET_SIZE && buf[0] == 0x05 && buf[7] == 0xFF) {
        pkt.valid  = true;
        pkt.state  = buf[1];
        pkt.target = buf[2];
        pkt.action = (char)buf[3];
    }
    return pkt;
}

ItemListPacket parse_item_list_packet(const unsigned char *buf, int n) {
    ItemListPacket pkt = { .valid = false, .count = 0 };
    if (n < 4 || buf[0] != 0x07) return pkt;

    int num = buf[1];
    int expected_len = 3 + 2 * num;
    if (num <= 0 || num > MAX_ITEMS || n != expected_len || buf[n - 1] != 0xFF) {
        printf("[!] Invalid item-list packet (n=%d, count=%d)\n", n, num);
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
            printf("[!] Skipping invalid item edge (%d, %d)\n", iu, iv);
        }
    }
    return pkt;
}

StyrResponse parse_styr_response(const unsigned char *buf, int n) {
    StyrResponse resp = { .valid = false, .action_done = 0 };
    if (n == PACKET_SIZE) {
        resp.valid       = true;
        resp.action_done = buf[4]; // Byte 4: action_done flagga
    }
    return resp;
}

SensorData parse_sensor_packet(const unsigned char *buf) {
    SensorData s;
    s.flags    = buf[0];
    s.line_var = buf[1];
    s.angle    = buf[2];
    s.gyro1    = buf[6];
    s.gyro2    = buf[7];
    return s;
}

// =================================================================
// BUILDING: Utgående paket
// =================================================================

void build_motor_packet(unsigned char out[PACKET_SIZE],
                        uint8_t state, bool is_pickup,
                        char command,
                        uint8_t line_var, uint8_t gyro1, uint8_t gyro2) {
    out[0] = 0x05;
    out[1] = state;
    out[2] = is_pickup ? 0x01 : 0x00;
    out[3] = (unsigned char)command;
    out[4] = line_var;
    out[5] = gyro1;
    out[6] = gyro2;
    out[7] = 0xFF;
}

void build_telemetry_packet(unsigned char out[PACKET_SIZE + 6],
                            uint8_t phase, char action, char next_action,
                            uint8_t line_var, uint8_t gyro1, uint8_t gyro2,
                            uint8_t flags, uint8_t node,
                            uint8_t item_idx, uint8_t item_count,
                            char direction, uint8_t action_done) {
    out[0]  = 0x06;
    out[1]  = phase;
    out[2]  = (unsigned char)action;
    out[3]  = (unsigned char)next_action;
    out[4]  = line_var;
    out[5]  = gyro1;
    out[6]  = gyro2;
    out[7]  = flags;
    out[8]  = node;
    out[9]  = item_idx;
    out[10] = item_count;
    out[11] = (unsigned char)direction;
    out[12] = action_done;
    out[13] = 0xFF;
}

int build_route_packet(unsigned char *out, const int *route, int max_nodes) {
    int count = 0;
    while (count < max_nodes && route[count] != STOP) count++;

    out[0] = 0x08;
    out[1] = (unsigned char)count;
    for (int i = 0; i < count; i++) {
        out[2 + i] = (unsigned char)route[i];
    }
    out[2 + count] = 0xFF;
    return 3 + count; // Total packet length
}

// =================================================================
// LOGGING
// =================================================================

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
