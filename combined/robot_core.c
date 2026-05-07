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
char pending_rotation_cmd = ' '; // Sparar rot-riktning medan den stannar
bool is_picking_up = false;
char pickup_cmd    = 'v'; // Upphämtningskommando: 'v' (vänster) eller 'h' (höger)
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
int telemetry_counter = 0;
bool route_changed    = false; // Flagga: ny rutt ska skickas till GUI

// --- LIVE STATE ---
char nasta_beslut  = 's';
char aktivt_beslut = 's';
int  loop_counter  = 0;
uint8_t current_node = START;
char current_dir = 's';
uint8_t action_done = 0;
bool rotation_done = false;
bool pickup_step_done = false;

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

// =================================================================
// LOGGNING FÖR STYRMODUL (med datum/tid)
// =================================================================
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
// STATE MACHINE: Beslutsfunktion och autonom startsekvens
// =================================================================
void aktivt_beslut_fn(int index) {
    if (current_phase == PHASE_TO_ITEM) {
        aktivt_beslut = beslut_till_vara[index];
        if (aktivt_beslut == 'e' || aktivt_beslut == 'o') {
            nasta_beslut = 'f';
        } else if (aktivt_beslut == 'X') {
            nasta_beslut = pickup_cmd;
        } else {
            nasta_beslut = beslut_till_vara[index + 1];
        }
    } else if (current_phase == PHASE_PICKUP) {
        aktivt_beslut = pickup_cmd;
        if (current_item_index + 1 < item_count) {
            nasta_beslut = 'f';
        } else {
            nasta_beslut = beslut_hem[0];
        }
    } else if (current_phase == PHASE_TO_HOME) {
        aktivt_beslut = beslut_hem[index];
        if (aktivt_beslut == 'e' || aktivt_beslut == 'o') {
            nasta_beslut = 'f';
        } else {
            nasta_beslut = beslut_hem[index + 1];
        }
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

    if (aktivt_beslut == 'e' || aktivt_beslut == 'o') {
        is_rotating = true;
        pending_rotation_cmd = aktivt_beslut; // Spara svängen
        aktivt_beslut = 's';                  // Skicka stop först (från stillastående)
        action_timer_start = current_time_ms();
    } else if (sim_sensor) {
        sim_segment_timer = current_time_ms();
    }

    log_next_action = true;
    route_changed  = true; // Skicka rutten till GUI
    printf("-> Route Calculated. Driving to item 1/%d...\n", item_count);
    if (sim_sensor) printf("[SIM] Intersections will be triggered every %d ms\n", SIM_SEGMENT_MS);
}

// =================================================================
// REFAKTORERING: Hjälpfunktioner för main
// =================================================================

void init_system(SystemPointers *sys) {
    init_karta();

    FILE *clr = fopen(VERIFY_LOG_FILE, "w");
    if (clr) fclose(clr);
    printf("--- PI CORE: DUAL I2C (0x10 & 0x12) + UDP ROUTER ---\n");

    // --- I2C: Motorstyrning (0x12) ---
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

    // --- I2C: Sensorkort (0x10) ---
    sys->i2c_sens_fd = open(I2C_DEVICE, O_RDWR);
    if (sys->i2c_sens_fd >= 0) {
        ioctl(sys->i2c_sens_fd, I2C_SLAVE, SENSOR_ADDR);
        if (write(sys->i2c_sens_fd, NULL, 0) < 0) {
            sim_sensor = true;
            printf("[SIM] Sensor Board (0x10) missing. Using time-based intersection simulation (%d ms).\n", SIM_SEGMENT_MS);
        } else {
            printf("Connected to Sensor Board (0x10)\n");
        }
    } else {
        sim_sensor = true;
        printf("[SIM] Could not open I2C for Sensor Board. Using time-based intersection simulation (%d ms).\n", SIM_SEGMENT_MS);
    }

    // --- UDP Socket ---
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
    memset(&sys->cliaddr, 0, sizeof(sys->cliaddr)); // Zero it out initially

    printf("Listening for UDP on port %d...\n\n", UDP_PORT);
}

void update_sensors(SystemPointers *sys, uint8_t *line_var_f, uint8_t *line_var_b, uint8_t *angle, uint8_t *gyro1, uint8_t *gyro2, uint8_t *flags, uint8_t *flags_korsning, uint8_t *flags_ny_korsning) {
    unsigned char sensor_raw[PACKET_SIZE];
    if (!sim_sensor && sys->i2c_sens_fd >= 0 && read(sys->i2c_sens_fd, sensor_raw, PACKET_SIZE) == PACKET_SIZE) { 
        
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
        
        if (!(*flags_ny_korsning)) {
            *flags_ny_korsning = (*flags & 0x20) >> 4; 
        }

        if (*flags_korsning == 1)      pickup_cmd = 'v';
        else if (*flags_korsning == 3) pickup_cmd = 'h';
    }
}

void process_network_packets(SystemPointers *sys, uint8_t line_var_f, uint8_t line_var_b, uint8_t gyro1, uint8_t gyro2) {
    unsigned char buffer[BUFFER_SIZE];
    int n = recvfrom(sys->sockfd, buffer, BUFFER_SIZE, MSG_DONTWAIT, (struct sockaddr *)&sys->cliaddr, &sys->cliaddr_len);
    
    if (n > 0) {
        gui_known = true;
    }

    // --- 0x05 Command Packet ---
    CommandPacket cmd = parse_command_packet(buffer, n);
    if (cmd.valid) {
        if (cmd.state == 0x00 || cmd.state == 0x01) {
            if (cmd.action == 'f' && current_phase == PHASE_IDLE) {
                start_autonomous_sequence(cmd.state);
            } else {
                unsigned char fwd[PACKET_SIZE];
                build_motor_packet(fwd, cmd.state, false, cmd.action, line_var_f, line_var_b, gyro1, gyro2);
                fwd[2] = cmd.target; // Behåll target-byte från GUI
                if (!sim_motor) write(sys->i2c_styr_fd, fwd, PACKET_SIZE);
                log_verification(fwd, cmd.action);
                printf("-> Manual Command Forwarded: '%c'\n", cmd.action);
            }
        } else if (cmd.state == 0x02 || cmd.state == 0x03) {
            if (current_phase != PHASE_IDLE) {
                printf("\n[!] MANUAL OVERRIDE DETECTED. Canceling Auto Route.\n");
                current_phase        = PHASE_IDLE;
                is_rotating          = false;
                is_picking_up        = false;
                is_dropping          = false;
                current_action_index = 0;
                korsning_aktiv       = 0;
                aktivt_beslut        = 's';
                nasta_beslut         = 's';
            }
            unsigned char fwd[PACKET_SIZE];
            build_motor_packet(fwd, cmd.state, false, cmd.action, line_var_f, line_var_b, gyro1, gyro2);
            fwd[2] = cmd.target;
            if (!sim_motor) write(sys->i2c_styr_fd, fwd, PACKET_SIZE);
            log_verification(fwd, cmd.action);
            printf("-> Manual Command Forwarded: '%c'\n", cmd.action);
        }
    }

    // --- 0x07 Item List Packet ---
    ItemListPacket items = parse_item_list_packet(buffer, n);
    if (items.valid && items.count > 0) {
        item_count = items.count;
        memcpy(item_list_u, items.items_u, items.count * sizeof(uint8_t));
        memcpy(item_list_v, items.items_v, items.count * sizeof(uint8_t));
        current_item_index = 0;
        printf("[ITEMS] Received %d valid item(s) from GUI\n", item_count);
    }
}

void process_autonomous_state(SystemPointers *sys, uint8_t line_var_f, uint8_t line_var_b, uint8_t gyro1, uint8_t gyro2, uint8_t flags_korsning, uint8_t *flags_ny_korsning) {
    if (current_phase == PHASE_IDLE) return;
    
    long long elapsed_in_state = current_time_ms() - action_timer_start;

    if (is_rotating) {
        if (sim_motor) {
            rotation_done = ((aktivt_beslut == 's' || aktivt_beslut == 'z') && elapsed_in_state >= 1000) ||
                            ((aktivt_beslut == 'e' || aktivt_beslut == 'o') && elapsed_in_state >= 2000);
        } 
        else if (elapsed_in_state > 300 && sys->i2c_styr_fd >= 0 && !rotation_done) {
            static long long last_rot_read = 0;
            if (current_time_ms() - last_rot_read > 50) {
                last_rot_read = current_time_ms();
                
                unsigned char styr_raw[PACKET_SIZE] = {0};
                if (read(sys->i2c_styr_fd, styr_raw, PACKET_SIZE) == PACKET_SIZE) {
                    
                    static unsigned char last_styr_rot[PACKET_SIZE] = {0};
                    if (memcmp(styr_raw, last_styr_rot, PACKET_SIZE) != 0) {
                        log_styr_response(styr_raw);
                        memcpy(last_styr_rot, styr_raw, PACKET_SIZE);
                    }
                    
                    StyrResponse resp = parse_styr_response(styr_raw, PACKET_SIZE);
                    if (resp.action_done == 1) {
                        rotation_done = true;
                    }
                }
            }
        }

        if (rotation_done && (aktivt_beslut == 's' || aktivt_beslut == 'z')) {
            aktivt_beslut = pending_rotation_cmd; 
            rotation_done = false; 
            action_timer_start = current_time_ms(); 
            log_next_action = true;
        }
        else if (rotation_done && (aktivt_beslut == 'e' || aktivt_beslut == 'o')) {
            is_rotating   = false;
            aktivt_beslut = 'f';
            rotation_done = false;
            korsning_aktiv = 1; 
            
            if (current_phase == PHASE_TO_ITEM) {
                nasta_beslut = beslut_till_vara[current_action_index + 1];
            } else if (current_phase == PHASE_TO_HOME) {
                nasta_beslut = beslut_hem[current_action_index + 1];
            }
            log_next_action = true;

            if (sim_sensor) sim_segment_timer = current_time_ms();
        }
    } 
    else if (is_picking_up) {
        if (sim_motor) {
            pickup_step_done = (aktivt_beslut == 'x' && elapsed_in_state >= 1500) ||
                               (aktivt_beslut == 'v' && elapsed_in_state >= 3000);
        } 
        else if (elapsed_in_state > 300 && sys->i2c_styr_fd >= 0 && !pickup_step_done) {
            static long long last_pickup_read = 0;
            if (current_time_ms() - last_pickup_read > 50) {
                last_pickup_read = current_time_ms();
                
                unsigned char styr_raw[PACKET_SIZE] = {0};
                if (read(sys->i2c_styr_fd, styr_raw, PACKET_SIZE) == PACKET_SIZE) {
                    
                    static unsigned char last_styr_pick[PACKET_SIZE] = {0};
                    if (memcmp(styr_raw, last_styr_pick, PACKET_SIZE) != 0) {
                        log_styr_response(styr_raw);
                        memcpy(last_styr_pick, styr_raw, PACKET_SIZE);
                    }
                    
                    StyrResponse resp = parse_styr_response(styr_raw, PACKET_SIZE);
                    if (resp.action_done == 1) {
                        pickup_step_done = true;
                    }
                }
            }
        }

        if (pickup_step_done && aktivt_beslut == 'x') {
            aktivt_beslut = pickup_cmd;
            pickup_step_done = false; 
            action_timer_start = current_time_ms(); 
            if (current_item_index + 1 < item_count) {
                nasta_beslut = 'f';
            } else {
                nasta_beslut = beslut_hem[0];
            }
            log_next_action = true;
        }
        else if (pickup_step_done && (aktivt_beslut == 'v' || aktivt_beslut == 'h')) {
            is_picking_up = false;
            current_item_index++;
            pickup_step_done = false;

            if (current_item_index < item_count) {
                vara_u = item_list_u[current_item_index];
                vara_v = item_list_v[current_item_index];
                printf("\n-> Item %d/%d: edge %d <-> %d\n", current_item_index + 1, item_count, vara_u, vara_v);
                planera_nasta_vara();
                planera_hem_fran_pickup();

                current_phase        = PHASE_TO_ITEM;
                current_action_index = 0;
                aktivt_beslut_fn(current_action_index);

                if (aktivt_beslut == 'e' || aktivt_beslut == 'o') {
                    is_rotating = true;
                    pending_rotation_cmd = aktivt_beslut;
                    aktivt_beslut = 's';
                    action_timer_start = current_time_ms();
                } else if (sim_sensor) {
                    sim_segment_timer = current_time_ms();
                }
                printf("-> PHASE CHANGE: Driving to item %d/%d...\n", current_item_index + 1, item_count);
                log_next_action = true;
                route_changed   = true;
            } else {
                planera_hem_fran_pickup();
                current_phase        = PHASE_TO_HOME;
                current_action_index = 0;
                aktivt_beslut_fn(current_action_index);

                if (aktivt_beslut == 'e' || aktivt_beslut == 'o') {
                    is_rotating = true;
                    pending_rotation_cmd = aktivt_beslut;
                    aktivt_beslut = 's';
                    action_timer_start = current_time_ms();
                } else if (sim_sensor) {
                    sim_segment_timer = current_time_ms();
                }
                printf("\n-> PHASE CHANGE: All %d items collected. Heading Home...\n", item_count);
                log_next_action = true;
                route_changed   = true;
            }
        }
    }
    // --- DROP STATE MACHINE ---
    else if (is_dropping) {
        if (sim_motor) {
            drop_step_done = ((aktivt_beslut == 's' || aktivt_beslut == 'z') && elapsed_in_state >= 1500) ||
                              (aktivt_beslut == 'w' && elapsed_in_state >= 3000);
        }
        else if (elapsed_in_state > 300 && sys->i2c_styr_fd >= 0 && !drop_step_done) {
            static long long last_drop_read = 0;
            if (current_time_ms() - last_drop_read > 50) {
                last_drop_read = current_time_ms();
                unsigned char styr_raw[PACKET_SIZE] = {0};
                if (read(sys->i2c_styr_fd, styr_raw, PACKET_SIZE) == PACKET_SIZE) {
                    StyrResponse resp = parse_styr_response(styr_raw, PACKET_SIZE);
                    if (resp.action_done == 1) {
                        drop_step_done = true;
                    }
                }
            }
        }

        if (drop_step_done && (aktivt_beslut == 's' || aktivt_beslut == 'z')) {
            aktivt_beslut = 'w';
            drop_step_done = false;
            action_timer_start = current_time_ms();
            nasta_beslut = 's';
            log_next_action = true;
        }
        else if (drop_step_done && aktivt_beslut == 'w') {
            is_dropping = false;
            drop_step_done = false;
            current_phase = PHASE_IDLE;
            current_node  = START;
            aktivt_beslut = 's';
            nasta_beslut  = 's';
            printf("\n=== AUTONOMOUS ROUTE COMPLETE ===\n\n");

            unsigned char stop_pkt[PACKET_SIZE];
            build_motor_packet(stop_pkt, current_auto_state, false, 's', line_var_f, line_var_b, gyro1, gyro2);
            if (!sim_motor) write(sys->i2c_styr_fd, stop_pkt, PACKET_SIZE);
        }
    } 
    else {
        // --- INTERSECTION DETECTION ---
        bool intersection_triggered = false;

        if (sim_sensor) {
            if (sim_segment_timer > 0 && (current_time_ms() - sim_segment_timer) >= SIM_SEGMENT_MS) {
                intersection_triggered = true;
                sim_segment_timer = 0;
            }
        } else {
            bool real_intersection = (flags_korsning == 2);
            bool pickup_marker     = ((flags_korsning == 1 || flags_korsning == 3) && nasta_beslut == 'X');
            
            if ((real_intersection || pickup_marker) && !korsning_aktiv) {
                intersection_triggered = true;
                korsning_aktiv    = 1;
                *flags_ny_korsning = 0;
            } else if (flags_korsning == 0) {
                korsning_aktiv = 0;
            }
        }

        if (intersection_triggered) {
            char previous_action = aktivt_beslut;

            current_action_index++;
            aktivt_beslut_fn(current_action_index);
            action_timer_start = current_time_ms();

            if (current_phase == PHASE_TO_ITEM) {
                current_node = rutt_till_vara[current_action_index];
                if (rutt_till_vara[current_action_index + 1] != STOP) {
                    current_dir = nodriktningsmatris[rutt_till_vara[current_action_index]][rutt_till_vara[current_action_index + 1]];
                }
            } else if (current_phase == PHASE_TO_HOME) {
                current_node = rutt_hem[current_action_index];
                if (rutt_hem[current_action_index + 1] != STOP) {
                    current_dir = nodriktningsmatris[rutt_hem[current_action_index]][rutt_hem[current_action_index + 1]];
                }
            }

            if (aktivt_beslut == 'e' || aktivt_beslut == 'o') {
                is_rotating = true;
                pending_rotation_cmd = aktivt_beslut; 
                aktivt_beslut = (previous_action == 'b') ? 'z' : 's'; 
                log_next_action = true;
            }
            else if (aktivt_beslut == 'X') {
                if (current_phase == PHASE_TO_ITEM) {
                    current_phase = PHASE_PICKUP;
                    aktivt_beslut = 'x'; 
                    nasta_beslut  = pickup_cmd;
                    is_picking_up = true;
                    printf("\n-> Pickup item %d/%d...\n", current_item_index + 1, item_count);
                    log_next_action = true;
                }
                else if (current_phase == PHASE_TO_HOME) {
                    current_phase = PHASE_DROP;
                    aktivt_beslut = (previous_action == 'b') ? 'z' : 's';
                    nasta_beslut = 'w';
                    is_dropping = true;
                    action_timer_start = current_time_ms();
                    printf("\n-> Dropping basket at home...\n");
                    log_next_action = true;
                }
            }
            else {
                log_next_action = true;
            }

            if (sim_sensor && !is_rotating && !is_picking_up && !is_dropping && aktivt_beslut != 'X') {
                sim_segment_timer = current_time_ms();
            }
        }
    }

    // --- SEND MOTOR COMMANDS ---
    if (current_phase != PHASE_IDLE && aktivt_beslut != 'X') {
        char skickat_kommando = aktivt_beslut;
        bool pickup_flag = (current_phase == PHASE_PICKUP && (aktivt_beslut == 'v' || aktivt_beslut == 'h')) ||
                           (current_phase == PHASE_DROP && aktivt_beslut == 'w');
        unsigned char auto_packet[PACKET_SIZE];
        build_motor_packet(auto_packet, current_auto_state, pickup_flag, skickat_kommando, line_var_f, line_var_b, gyro1, gyro2);

        if (!sim_motor) write(sys->i2c_styr_fd, auto_packet, PACKET_SIZE);
        
        if (log_next_action) {
            printf("Action updated to: '%c' (Sending to motors: '%c', Index: %d, Next: '%c')\n",
                   aktivt_beslut, auto_packet[3], current_action_index, nasta_beslut);
            log_next_action = false;
        }

        static int blasting_log_counter = 0;
        blasting_log_counter++;
        if (blasting_log_counter >= 50) { 
            log_verification(auto_packet, auto_packet[3]);
            blasting_log_counter = 0;
        }
    }
}

void send_telemetry_and_routes(SystemPointers *sys, uint8_t line_var_f, uint8_t gyro1, uint8_t gyro2, uint8_t flags) {
    if (gui_known) {
        telemetry_counter++;
        if (telemetry_counter >= 50) {
            unsigned char tpkt[PACKET_SIZE + 6];
            build_telemetry_packet(tpkt,
                (uint8_t)current_phase, aktivt_beslut, nasta_beslut,
                line_var_f, gyro1, gyro2, flags, current_node,
                (uint8_t)current_item_index, (uint8_t)item_count,
                current_dir, action_done);
            sendto(sys->sockfd, tpkt, (PACKET_SIZE + 6), 0, (struct sockaddr *)&sys->cliaddr, sizeof(sys->cliaddr));
            telemetry_counter = 0;
        }
    }
    
    if (gui_known && route_changed) {
        route_changed = false;
        int *rutt = (current_phase == PHASE_TO_HOME) ? rutt_hem : rutt_till_vara;
        unsigned char rpkt[NODES + 3];
        int rpkt_len = build_route_packet(rpkt, rutt, NODES);
        sendto(sys->sockfd, rpkt, rpkt_len, 0, (struct sockaddr *)&sys->cliaddr, sizeof(sys->cliaddr));
    }
}

// =================================================================
// HUVUDPROGRAM
// =================================================================
int main() {
    SystemPointers sys;
    uint8_t line_var_f = 0, line_var_b = 0, angle = 0, gyro1 = 0, gyro2 = 0;
    uint8_t flags = 0, flags_korsning = 0, flags_ny_korsning = 0;

    init_system(&sys);

    if (sim_sensor || sim_motor) {
        printf("\n*** RUNNING IN SIM MODE ***\n\n");
    }

    // =============================================================
    // NON-BLOCKING MAIN LOOP (~500 Hz)
    // =============================================================
    while (1) {
        update_sensors(&sys, &line_var_f, &line_var_b, &angle, &gyro1, &gyro2, &flags, &flags_korsning, &flags_ny_korsning);
        process_network_packets(&sys, line_var_f, line_var_b, gyro1, gyro2);
        process_autonomous_state(&sys, line_var_f, line_var_b, gyro1, gyro2, flags_korsning, &flags_ny_korsning);
        send_telemetry_and_routes(&sys, line_var_f, gyro1, gyro2, flags);
        
        // TINY DELAY (2ms för 500Hz loop)
        usleep(25000); 
    }

    // Cleanup
    close(sys.sockfd);
    if (sys.i2c_styr_fd >= 0) close(sys.i2c_styr_fd);
    if (sys.i2c_sens_fd >= 0) close(sys.i2c_sens_fd);
    
    return 0;
}