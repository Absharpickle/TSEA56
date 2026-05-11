#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <stdint.h>
#include <time.h>

#include "pathfinding.h"
#include "protocol.h"

// --- SIM MODE DEFINITIONS ---
#define SIM_SEGMENT_MS 1000

// --- STATE MACHINE ---
typedef enum {
    PHASE_IDLE = 0,
    PHASE_TO_ITEM,
    PHASE_PICKUP,
    PHASE_TO_HOME,
    PHASE_DROP
} AutoPhase;

AutoPhase current_phase      = PHASE_IDLE;
int current_action_index     = 0;
unsigned char current_auto_state = 1;
bool log_next_action         = false;

bool is_rotating   = false;
char pending_rotation_cmd = ' ';
bool is_picking_up = false;
char pickup_cmd    = 'v';
bool is_dropping   = false;
bool drop_step_done = false;
long long action_timer_start = 0;
uint8_t korsning_aktiv = 0;

// --- SIM MODE GLOBALS ---
bool sim_sensor = false;
bool sim_motor  = false;
long long sim_segment_timer = 0;

// --- TELEMETRY GLOBALS ---
bool gui_known        = false;
uint32_t telemetry_counter = 0;  // FIX: uint32_t för att undvika int-overflow
bool route_changed    = false;

// --- LIVE STATE ---
char nasta_beslut  = 's';
char aktivt_beslut = 's';
int  loop_counter  = 0;
uint8_t current_node = START;
char current_dir = 's';
uint8_t action_done = 0;
bool rotation_done = false;
bool pickup_step_done = false;

// --- HINDERHANTERING ---
bool is_handling_obstacle = false;
long long obstacle_timer_start = 0;

// --- SYSTEM POINTERS STRUCT ---
typedef struct {
    int sockfd;
    int i2c_styr_fd;
    int i2c_sens_fd;
    struct sockaddr_in cliaddr;
    socklen_t cliaddr_len;
} SystemPointers;

// =================================================================
// HJÄLPFUNKTION: Tidsmätning i millisekunder
// =================================================================
long long current_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

void log_styr_response(const unsigned char *received) {
    FILE *f = fopen(VERIFY_LOG_FILE, "a");
    if (f == NULL) return;
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    fprintf(f, "[%02d:%02d:%02d] STYR SVAR (0x12): ", t->tm_hour, t->tm_min, t->tm_sec);
    for (int i = 0; i < PACKET_SIZE; i++) fprintf(f, "%02X ", received[i]);
    fprintf(f, "\n\n");
    fclose(f);
}

// =================================================================
// FIX #2: Bounds-check på index+1 i aktivt_beslut_fn
// Läs alltid nästa beslut ur arrayen för nasta_beslut,
// även vid 'e'/'o' – så att nasta_beslut är korrekt efter rotation klar.
// =================================================================
void aktivt_beslut_fn(int index) {
    if (current_phase == PHASE_TO_ITEM) {
        aktivt_beslut = beslut_till_vara[index];
        if (aktivt_beslut == 'X') {
            nasta_beslut = pickup_cmd;
        } else {
            nasta_beslut = (index + 1 < NODES) ? beslut_till_vara[index + 1] : 's';
        }
    } else if (current_phase == PHASE_PICKUP) {
        aktivt_beslut = pickup_cmd;
        if (current_item_index + 1 < item_count) {
            nasta_beslut = 'f';
        } else {
            nasta_beslut = (beslut_hem[0] != '\0') ? beslut_hem[0] : 's';
        }
    } else if (current_phase == PHASE_TO_HOME) {
        aktivt_beslut = beslut_hem[index];
        nasta_beslut = (index + 1 < NODES) ? beslut_hem[index + 1] : 's';
    }
}

// =================================================================
// Hjälpfunktion: starta rotation om nästa beslut kräver sväng,
// annars logga direkt.
// Obs: 'X' och 'b' hanteras separat i korsningslogiken med rätt
// stoppkommando ('x' resp. 'z') – når aldrig hit för dessa värden.
// =================================================================
static void maybe_start_rotation_or_log() {
    if (aktivt_beslut == 'e' || aktivt_beslut == 'o') {
        is_rotating = true;
        pending_rotation_cmd = aktivt_beslut;
        aktivt_beslut = 's';
        action_timer_start = current_time_ms();
    } else {
        log_next_action = true;
    }
}

void start_autonomous_sequence(unsigned char state) {
    if (item_count <= 0) {
        printf("[!] No items configured. Send item list (0x07) first.\n");
        return;
    }

    current_item_index = 0;
    vara_u = item_list_u[0];
    vara_v = item_list_v[0];

    printf("\n=== AUTONOMOUS ROUTE: %d item(s) to collect ===\n", item_count);
    printf("-> Item 1/%d: edge %d <-> %d\n", item_count, vara_u, vara_v);
    planera_till_vara(START, 's');
    planera_hem_fran_pickup();

    current_auto_state   = state;
    current_phase        = PHASE_TO_ITEM;
    current_action_index = 0;
    korsning_aktiv       = 0;
    loop_counter         = 0;

    aktivt_beslut_fn(current_action_index);
    current_node = rutt_till_vara[0];
    current_dir  = 's';

    maybe_start_rotation_or_log();

    if (sim_sensor && !is_rotating) {
        sim_segment_timer = current_time_ms();
    }

    log_next_action = true;
    route_changed   = true;
    printf("-> Route Calculated. Driving to item 1/%d...\n", item_count);
    if (sim_sensor) printf("[SIM] Intersections will be triggered every %d ms\n", SIM_SEGMENT_MS);
}

void init_system(SystemPointers *sys) {
    init_karta();

    FILE *clr = fopen(VERIFY_LOG_FILE, "w");
    if (clr) fclose(clr);
    printf("--- PI CORE: DUAL I2C (0x10 & 0x12) + UDP ROUTER ---\n");

    // FIX: Öppna separata file descriptors för styr respektive sensor
    sys->i2c_styr_fd = open(I2C_DEVICE, O_RDWR);
    if (sys->i2c_styr_fd >= 0) {
        ioctl(sys->i2c_styr_fd, I2C_SLAVE, STYRKOMM_ADDR);
        if (write(sys->i2c_styr_fd, NULL, 0) < 0) {
            sim_motor = true;
            printf("[SIM] Motor Controller (0x12) missing. Motor writes disabled.\n");
        } else {
            printf("Connected to Motor Controller (0x12)\n");
        }
    } else {
        sim_motor = true;
        printf("[SIM] Could not open I2C for Motor Controller. Motor writes disabled.\n");
    }

    // FIX: Ny open()-anrop för sensorn så att fd inte delar instans med styrfd
    sys->i2c_sens_fd = open(I2C_DEVICE, O_RDWR);
    if (sys->i2c_sens_fd >= 0) {
        // FIX: Säkerställ att vi verkligen sätter sensor-adressen på den nya fd:n
        if (ioctl(sys->i2c_sens_fd, I2C_SLAVE, SENSOR_ADDR) < 0) {
            close(sys->i2c_sens_fd);
            sys->i2c_sens_fd = -1;
            sim_sensor = true;
            printf("[SIM] Could not set I2C slave address for Sensor Board.\n");
        } else if (write(sys->i2c_sens_fd, NULL, 0) < 0) {
            sim_sensor = true;
            printf("[SIM] Sensor Board (0x10) missing.\n");
        } else {
            printf("Connected to Sensor Board (0x10)\n");
        }
    } else {
        sim_sensor = true;
        printf("[SIM] Could not open I2C for Sensor Board.\n");
    }

    if ((sys->sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    int opt = 1;
    setsockopt(sys->sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port        = htons(UDP_PORT);

    if (bind(sys->sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }

    sys->cliaddr_len = sizeof(sys->cliaddr);
    memset(&sys->cliaddr, 0, sizeof(sys->cliaddr));

    printf("Listening for UDP on port %d...\n\n", UDP_PORT);
}

void update_sensors(SystemPointers *sys, uint8_t *line_var_f, uint8_t *line_var_b,
                    uint8_t *angle, uint8_t *gyro1, uint8_t *gyro2,
                    uint8_t *flags, uint8_t *flags_korsning,
                    uint8_t *flags_ny_korsning, bool *obstacle_detected) {

    unsigned char sensor_raw[PACKET_SIZE];
    if (!sim_sensor && sys->i2c_sens_fd >= 0 &&
        read(sys->i2c_sens_fd, sensor_raw, PACKET_SIZE) == PACKET_SIZE) {

        static unsigned char last_sensor_packet[PACKET_SIZE] = {0};
        if (memcmp(sensor_raw, last_sensor_packet, PACKET_SIZE) != 0) {
            log_sensor_data(sensor_raw);
            memcpy(last_sensor_packet, sensor_raw, PACKET_SIZE);
        }

        SensorData sd = parse_sensor_packet(sensor_raw);
        *flags      = sd.flags;
        *line_var_f = sd.line_var_f;
        *line_var_b = sd.line_var_b;
        *angle      = sd.angle;
        *gyro1      = sd.gyro1;
        *gyro2      = sd.gyro2;

        *flags_korsning = (*flags & 0x0C) >> 2;

        // Bit 5 (0x20): ny korsningshändelse – sätts bara om den inte redan är aktiv
        if (!(*flags_ny_korsning)) {
            *flags_ny_korsning = (*flags & 0x20) >> 5;
        }

        // Bit 4 (0x10): hinder detekterat
        *obstacle_detected = (*flags & 0x10) != 0;

        // FIX #5: Spara pickup_cmd när vi är vid pickup-noden, inte löpande
        // pickup_cmd sätts här bara om vi faktiskt är på väg att plocka upp
        if (current_phase == PHASE_TO_ITEM && aktivt_beslut == 'X') {
            if (*flags_korsning == 1)      pickup_cmd = 'v';
            else if (*flags_korsning == 3) pickup_cmd = 'h';
        }
    }
}

void process_network_packets(SystemPointers *sys, uint8_t line_var_f, uint8_t line_var_b,
                              uint8_t gyro1, uint8_t gyro2) {
    unsigned char buffer[BUFFER_SIZE];
    int n = recvfrom(sys->sockfd, buffer, BUFFER_SIZE, MSG_DONTWAIT,
                     (struct sockaddr *)&sys->cliaddr, &sys->cliaddr_len);
    if (n > 0) gui_known = true;

    CommandPacket cmd = parse_command_packet(buffer, n);
    if (cmd.valid) {
        if (cmd.state == 0x00 || cmd.state == 0x01) {
            if (cmd.action == 'f' && current_phase == PHASE_IDLE) {
                start_autonomous_sequence(cmd.state);
            } else {
                unsigned char fwd[PACKET_SIZE];
                build_motor_packet(fwd, cmd.state, false, cmd.action,
                                   line_var_f, line_var_b, gyro1, gyro2);
                fwd[2] = cmd.target;
                if (!sim_motor) write(sys->i2c_styr_fd, fwd, PACKET_SIZE);
                log_verification(fwd, cmd.action);
                printf("-> Manual Command Forwarded: '%c'\n", cmd.action);
            }
        } else if (cmd.state == 0x02 || cmd.state == 0x03) {
            // FIX: Återställ item_count och listor vid override så att state är konsistent
            init_karta();
            item_count = 0;
            memset(item_list_u, 0, sizeof(item_list_u));
            memset(item_list_v, 0, sizeof(item_list_v));

            if (current_phase != PHASE_IDLE) {
                printf("\n[!] MANUAL OVERRIDE DETECTED. Map & items reset. Canceling Auto Route.\n");
                current_phase        = PHASE_IDLE;
                is_rotating          = false;
                is_picking_up        = false;
                is_dropping          = false;
                current_action_index = 0;
                korsning_aktiv       = 0;
                aktivt_beslut        = 's';
                nasta_beslut         = 's';
                is_handling_obstacle = false;
            }
            unsigned char fwd[PACKET_SIZE];
            build_motor_packet(fwd, cmd.state, false, cmd.action,
                               line_var_f, line_var_b, gyro1, gyro2);
            fwd[2] = cmd.target;
            if (!sim_motor) write(sys->i2c_styr_fd, fwd, PACKET_SIZE);
            log_verification(fwd, cmd.action);
        }
    }

    ItemListPacket item_pkt = parse_item_list_packet(buffer, n);
    if (item_pkt.valid) {
        item_count = item_pkt.count;
        memcpy(item_list_u, item_pkt.items_u, item_count);
        memcpy(item_list_v, item_pkt.items_v, item_count);
        printf("\n-> Received new item list (%d items)\n", item_count);
    }
}

// =================================================================
// FIX #1: Hjälpfunktion som triggar PHASE_DROP när hemrutten är slut
// =================================================================
static void start_drop_sequence(SystemPointers *sys, uint8_t line_var_f,
                                 uint8_t line_var_b, uint8_t gyro1, uint8_t gyro2) {
    // FIX #7: Verifiera att vi faktiskt är vid START-noden
    if (current_node != START) {
        printf("[!] VARNING: Försöker droppa men current_node=%d, förväntat START=%d\n",
               current_node, START);
    }

    current_phase  = PHASE_DROP;
    is_dropping    = true;
    drop_step_done = false;

    // Steg 1: Stanna roboten med 'z' (bromsande stopp) innan 'w' (drop)
    aktivt_beslut = 'z';
    nasta_beslut  = 'w';
    action_timer_start = current_time_ms();

    printf("\n-> Hemma vid nod %d! Stannar och skickar drop-kommando...\n", current_node);

    unsigned char stop_pkt[PACKET_SIZE];
    build_motor_packet(stop_pkt, current_auto_state, false, 'z',
                       line_var_f, line_var_b, gyro1, gyro2);
    if (!sim_motor) write(sys->i2c_styr_fd, stop_pkt, PACKET_SIZE);
    log_next_action = true;
}

void process_autonomous_state(SystemPointers *sys, uint8_t line_var_f, uint8_t line_var_b,
                               uint8_t gyro1, uint8_t gyro2, uint8_t flags_korsning,
                               uint8_t *flags_ny_korsning, bool obstacle_detected) {
    if (current_phase == PHASE_IDLE) return;

    long long elapsed_in_state = current_time_ms() - action_timer_start;

    // =================================================================
    // HINDERHANTERING (DELAY & OMKALKYLERING)
    // =================================================================
    if (is_handling_obstacle) {
        if (current_time_ms() - obstacle_timer_start >= 500) {
            is_handling_obstacle = false;
            aktivt_beslut = 'f';
            log_next_action = true;
        } else {
            unsigned char stop_pkt[PACKET_SIZE];
            build_motor_packet(stop_pkt, current_auto_state, false, 's',
                               line_var_f, line_var_b, gyro1, gyro2);
            if (!sim_motor) write(sys->i2c_styr_fd, stop_pkt, PACKET_SIZE);
            return;
        }
    }

    // Hinderdetektion
    if (obstacle_detected && !is_handling_obstacle && !is_rotating &&
        !is_picking_up && !is_dropping && korsning_aktiv == 0) {

        int approaching_node = -1;
        int *current_route  = (current_phase == PHASE_TO_ITEM) ? rutt_till_vara : rutt_hem;
        char *current_beslut = (current_phase == PHASE_TO_ITEM) ? beslut_till_vara : beslut_hem;

        if (current_action_index + 1 < NODES) {
            approaching_node = current_route[current_action_index + 1];
        }

        if (approaching_node >= 0 && approaching_node < NODES) {
            int blocked_node = -1;

            // FIX #3: Beräkna riktningen UT från approaching_node baserat på rutten,
            // inte current_dir som är riktningen IN till noden.
            char exit_dir = ' ';
            if (current_action_index + 2 < NODES) {
                int node_after = current_route[current_action_index + 2];
                if (node_after >= 0 && node_after < NODES) {
                    exit_dir = nodriktningsmatris[approaching_node][node_after];
                }
            }

            for (int i = 0; i < NODES; i++) {
                if (vag[approaching_node][i]) {
                    // Matcha med utgångsriktning om vi kunde ta reda på den
                    if (exit_dir != ' ') {
                        if (nodriktningsmatris[approaching_node][i] == exit_dir) {
                            blocked_node = i;
                            break;
                        }
                    } else {
                        // Fallback: använd current_dir om vi är på sista segmentet
                        if (nodriktningsmatris[approaching_node][i] == current_dir) {
                            blocked_node = i;
                            break;
                        }
                    }
                }
            }

            if (blocked_node != -1) {
                printf("\n[!] HINDER UPPTÄCKT! Tar bort väg %d <-> %d och räknar om rutt...\n",
                       approaching_node, blocked_node);

                vag[approaching_node][blocked_node] = 0;
                vag[blocked_node][approaching_node] = 0;

                if (current_phase == PHASE_TO_ITEM) {
                    planera_till_vara(approaching_node, current_dir);
                } else if (current_phase == PHASE_TO_HOME) {
                    int rutt_tmp[NODES];
                    hitta_rutt(approaching_node, START, rutt_tmp, current_dir);
                    memcpy(rutt_hem, rutt_tmp, sizeof(rutt_tmp));
                    bygg_beslut(rutt_hem, current_dir, beslut_hem);
                }

                current_action_index = -1;
                aktivt_beslut = 's';
                nasta_beslut  = current_beslut[0];
                route_changed = true;

                is_handling_obstacle = true;
                obstacle_timer_start = current_time_ms();

                unsigned char stop_pkt[PACKET_SIZE];
                build_motor_packet(stop_pkt, current_auto_state, false, 's',
                                   line_var_f, line_var_b, gyro1, gyro2);
                if (!sim_motor) write(sys->i2c_styr_fd, stop_pkt, PACKET_SIZE);
                return;
            }
        }
    }
    // =================================================================

    // --- ROTATIONSHANTERING ---
    if (is_rotating) {
        unsigned char rot_pkt[PACKET_SIZE];
        build_motor_packet(rot_pkt, current_auto_state, false, pending_rotation_cmd,
                           line_var_f, line_var_b, gyro1, gyro2);

        if (!sim_motor) {
            // Skriv kommandot FÖRST, läs sedan svaret nästa iteration
            write(sys->i2c_styr_fd, rot_pkt, PACKET_SIZE);
            unsigned char ack_buf[PACKET_SIZE];
            if (read(sys->i2c_styr_fd, ack_buf, PACKET_SIZE) == PACKET_SIZE) {
                log_styr_response(ack_buf);
                StyrResponse resp = parse_styr_response(ack_buf, PACKET_SIZE);
                if (resp.valid && resp.action_done == 1) {
                    rotation_done = true;
                }
            }
        } else {
            if (elapsed_in_state > 2000) rotation_done = true;
        }

        if (rotation_done) {
            is_rotating   = false;
            rotation_done = false;
            current_dir   = apply_turn(current_dir, pending_rotation_cmd);
            // Efter rotation klar: nästa beslut är redan sparat i nasta_beslut
            // (sattes av aktivt_beslut_fn när rotationen startades).
            // Återställ aktivt_beslut och kör vidare framåt.
            aktivt_beslut = nasta_beslut;
            log_next_action = true;
        }
        return;
    }

    // --- PICKUP-HANTERING ---
    if (is_picking_up) {
        unsigned char pick_pkt[PACKET_SIZE];
        build_motor_packet(pick_pkt, current_auto_state, true, pickup_cmd,
                           line_var_f, line_var_b, gyro1, gyro2);

        if (!sim_motor) {
            write(sys->i2c_styr_fd, pick_pkt, PACKET_SIZE);
            unsigned char ack_buf[PACKET_SIZE];
            if (read(sys->i2c_styr_fd, ack_buf, PACKET_SIZE) == PACKET_SIZE) {
                StyrResponse resp = parse_styr_response(ack_buf, PACKET_SIZE);
                if (resp.valid && resp.action_done == 1) {
                    pickup_step_done = true;
                }
            }
        } else {
            if (elapsed_in_state > 3000) pickup_step_done = true;
        }

        if (pickup_step_done) {
            is_picking_up    = false;
            pickup_step_done = false;
            current_item_index++;

            if (current_item_index < item_count) {
                vara_u = item_list_u[current_item_index];
                vara_v = item_list_v[current_item_index];
                planera_till_vara(current_node, current_dir);
                planera_hem_fran_pickup();

                current_phase        = PHASE_TO_ITEM;
                current_action_index = 0;
                aktivt_beslut_fn(current_action_index);
                route_changed = true;
                printf("\n-> Picked up item %d/%d! Next target edge: %d <-> %d\n",
                       current_item_index, item_count, vara_u, vara_v);
            } else {
                printf("\n-> All items collected! Returning home to node %d.\n", START);
                current_phase        = PHASE_TO_HOME;
                current_action_index = 0;
                aktivt_beslut_fn(current_action_index);
                route_changed = true;
            }

            maybe_start_rotation_or_log();
            if (sim_sensor && !is_rotating) {
                sim_segment_timer = current_time_ms();
            }
        }
        return;
    }

    // --- DROP-HANTERING ---
    if (is_dropping) {
        unsigned char drop_pkt[PACKET_SIZE];
        build_motor_packet(drop_pkt, current_auto_state, true, aktivt_beslut,
                           line_var_f, line_var_b, gyro1, gyro2);

        if (!sim_motor) {
            write(sys->i2c_styr_fd, drop_pkt, PACKET_SIZE);
            unsigned char ack_buf[PACKET_SIZE];
            if (read(sys->i2c_styr_fd, ack_buf, PACKET_SIZE) == PACKET_SIZE) {
                StyrResponse resp = parse_styr_response(ack_buf, PACKET_SIZE);
                if (resp.valid && resp.action_done == 1) {
                    drop_step_done = true;
                }
            }
        } else {
            if ((aktivt_beslut == 'z' || aktivt_beslut == 's') && elapsed_in_state >= 1500) {
                drop_step_done = true;
            } else if (aktivt_beslut == 'w' && elapsed_in_state >= 3000) {
                drop_step_done = true;
            }
        }

        if (drop_step_done) {
            if (aktivt_beslut == 'z' || aktivt_beslut == 's') {
                // Steg 1 klart: byt till drop-kommando 'w'
                aktivt_beslut  = 'w';
                drop_step_done = false;
                action_timer_start = current_time_ms();
                nasta_beslut = 's';
                log_next_action = true;
                printf("\n-> Robot stoppad. Skickar drop-kommando ('w')...\n");
            } else if (aktivt_beslut == 'w') {
                // Steg 2 klart: uppdraget avslutat
                is_dropping    = false;
                drop_step_done = false;

                current_phase = PHASE_IDLE;
                current_node  = START;
                aktivt_beslut = 's';
                nasta_beslut  = 's';

                printf("\n=== AUTONOMOUS ROUTE COMPLETE ===\n-> Varorna är lämnade och klorna öppna!\n\n");

                unsigned char stop_pkt[PACKET_SIZE];
                build_motor_packet(stop_pkt, current_auto_state, false, 's',
                                   line_var_f, line_var_b, gyro1, gyro2);
                if (!sim_motor) write(sys->i2c_styr_fd, stop_pkt, PACKET_SIZE);
            }
        }
        return;
    }

    // =================================================================
    // KORSNINGSLOGIK
    // FIX #4: Flaggan nollställs inuti blocket, inte i else-grenen,
    //         för att undvika att missa korsningar om sensorn läser snabbt.
    // =================================================================
    bool intersection_trigger = (*flags_ny_korsning == 1) ||
                                 (sim_sensor && current_time_ms() - sim_segment_timer > SIM_SEGMENT_MS);

    if (intersection_trigger && korsning_aktiv == 0) {
        // Konsumera flaggan omedelbart
        *flags_ny_korsning = 0;
        if (sim_sensor) sim_segment_timer = current_time_ms();

        korsning_aktiv = 1;
        current_action_index++;

        if (current_phase == PHASE_TO_ITEM) {
            if (current_action_index < NODES)
                current_node = rutt_till_vara[current_action_index];
        } else if (current_phase == PHASE_TO_HOME) {
            if (current_action_index < NODES)
                current_node = rutt_hem[current_action_index];
        }

        aktivt_beslut_fn(current_action_index);

        if (aktivt_beslut == 'X') {
            // Skicka 'x' som stoppkommando inför pickup
            unsigned char xstop_pkt[PACKET_SIZE];
            build_motor_packet(xstop_pkt, current_auto_state, false, 'x',
                               line_var_f, line_var_b, gyro1, gyro2);
            if (!sim_motor) write(sys->i2c_styr_fd, xstop_pkt, PACKET_SIZE);
            log_verification(xstop_pkt, 'x');
            is_picking_up = true;
            action_timer_start = current_time_ms();
        } else if (aktivt_beslut == 'b') {
            // Skicka 'z' som stoppkommando inför backning
            unsigned char zstop_pkt[PACKET_SIZE];
            build_motor_packet(zstop_pkt, current_auto_state, false, 'z',
                               line_var_f, line_var_b, gyro1, gyro2);
            if (!sim_motor) write(sys->i2c_styr_fd, zstop_pkt, PACKET_SIZE);
            log_verification(zstop_pkt, 'z');
            log_next_action = true;
        } else {
            maybe_start_rotation_or_log();
        }

        // FIX #1: Kolla om hemrutten är slut – trigga drop om vi är hemma
        if (current_phase == PHASE_TO_HOME) {
            // Rutten är slut när nästa beslut är 's' och vi nått START
            // beslut_hem-arrayen avslutas med en nollterminering eller sentinel
            bool home_route_done = false;
            if (current_action_index < NODES) {
                // Om beslutet vid denna nod är 's' och noden är START = framme
                home_route_done = (current_node == START && aktivt_beslut == 's');
            }
            if (home_route_done) {
                start_drop_sequence(sys, line_var_f, line_var_b, gyro1, gyro2);
                return;
            }
        }

    } else if (!intersection_trigger) {
        korsning_aktiv = 0;
    }

    // Skicka aktivt beslut var 5:e loop-iteration
    if (loop_counter % 5 == 0) {
        unsigned char m_pkt[PACKET_SIZE];
        build_motor_packet(m_pkt, current_auto_state, false, aktivt_beslut,
                           line_var_f, line_var_b, gyro1, gyro2);
        if (!sim_motor) write(sys->i2c_styr_fd, m_pkt, PACKET_SIZE);
        if (log_next_action) {
            log_verification(m_pkt, aktivt_beslut);
            log_next_action = false;
        }
    }
    loop_counter++;
}

void send_telemetry_and_routes(SystemPointers *sys, uint8_t line_var_f,
                                uint8_t gyro1, uint8_t gyro2, uint8_t flags) {
    if (!gui_known) return;

    // FIX: uint32_t räknare – räcker i 3+ år vid 40 Hz
    if (telemetry_counter++ % 25 == 0) {
        unsigned char tpkt[PACKET_SIZE + 6];
        build_telemetry_packet(tpkt, (uint8_t)current_phase, aktivt_beslut, nasta_beslut,
                               line_var_f, gyro1, gyro2, flags, current_node,
                               current_item_index, item_count, current_dir, action_done);
        sendto(sys->sockfd, tpkt, PACKET_SIZE + 6, 0,
               (struct sockaddr *)&sys->cliaddr, sys->cliaddr_len);
    }

    if (route_changed) {
        route_changed = false;
        int *rutt = (current_phase == PHASE_TO_ITEM) ? rutt_till_vara : rutt_hem;
        unsigned char rpkt[NODES + 3];
        int rpkt_len = build_route_packet(rpkt, rutt, NODES);
        sendto(sys->sockfd, rpkt, rpkt_len, 0,
               (struct sockaddr *)&sys->cliaddr, sys->cliaddr_len);
    }
}

int main() {
    SystemPointers sys;
    uint8_t line_var_f = 0, line_var_b = 0, angle = 0;
    uint8_t gyro1 = 0, gyro2 = 0;
    uint8_t flags = 0, flags_korsning = 0, flags_ny_korsning = 0;
    bool obstacle_detected = false;

    init_system(&sys);

    if (sim_sensor || sim_motor) {
        printf("\n*** RUNNING IN SIM MODE ***\n\n");
    }

    while (1) {
        update_sensors(&sys, &line_var_f, &line_var_b, &angle, &gyro1, &gyro2,
                       &flags, &flags_korsning, &flags_ny_korsning, &obstacle_detected);
        process_network_packets(&sys, line_var_f, line_var_b, gyro1, gyro2);
        process_autonomous_state(&sys, line_var_f, line_var_b, gyro1, gyro2,
                                 flags_korsning, &flags_ny_korsning, obstacle_detected);
        send_telemetry_and_routes(&sys, line_var_f, gyro1, gyro2, flags);

        usleep(25000);
    }
    return 0;
}