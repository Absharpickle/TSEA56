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

#include "pathfinding.h"
#include "protocol.h"

// --- SIM MODE DEFINITIONS ---
#define SIM_SEGMENT_MS 3000

// --- STATE MACHINE ---
typedef enum {
    PHASE_IDLE = 0,
    PHASE_TO_ITEM,
    PHASE_PICKUP,
    PHASE_TO_HOME
} AutoPhase;

AutoPhase current_phase      = PHASE_IDLE;
int current_action_index     = 0;
unsigned char current_auto_state = 1;
bool log_next_action         = false;

bool is_rotating   = false;
bool is_picking_up = false;
long long action_timer_start = 0;
uint8_t korsning_aktiv = 0;

// --- SIM MODE GLOBALS ---
bool sim_sensor = false;
bool sim_motor  = false;
long long sim_segment_timer = 0;

// --- TELEMETRY GLOBALS ---
bool gui_known        = false;
int telemetry_counter = 0;

// --- LIVE STATE ---
char nasta_beslut  = 's';
char aktivt_beslut = 's';
int  loop_counter  = 0;
uint8_t current_node = START;
char current_dir = 's';

// =================================================================
// HJÄLPFUNKTION: Tidsmätning i millisekunder
// =================================================================
long long current_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

// =================================================================
// STATE MACHINE: Beslutsfunktion och autonom startsekvens
// =================================================================
void aktivt_beslut_fn(int index) {
    if (current_phase == PHASE_TO_ITEM) {
        aktivt_beslut = beslut_till_vara[index];
        if (aktivt_beslut == 'e' || aktivt_beslut == 'o' || aktivt_beslut == 'u') {
            nasta_beslut = 'f';
        } else if (aktivt_beslut == 'X') {
            nasta_beslut = 'v';
        } else {
            nasta_beslut = beslut_till_vara[index + 1];
        }
    } else if (current_phase == PHASE_PICKUP) {
        aktivt_beslut = 'v';
        if (current_item_index + 1 < item_count) {
            nasta_beslut = 'f';
        } else {
            nasta_beslut = beslut_hem[0];
        }
    } else if (current_phase == PHASE_TO_HOME) {
        aktivt_beslut = beslut_hem[index];
        if (aktivt_beslut == 'e' || aktivt_beslut == 'o' || aktivt_beslut == 'u') {
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

    if (aktivt_beslut == 'e' || aktivt_beslut == 'o' || aktivt_beslut == 'u') {
        is_rotating = true;
        action_timer_start = current_time_ms();
    } else if (sim_sensor) {
        sim_segment_timer = current_time_ms();
    }

    log_next_action = true;
    printf("-> Route Calculated. Driving to item 1/%d...\n", item_count);
    if (sim_sensor) printf("[SIM] Intersections will be triggered every %d ms\n", SIM_SEGMENT_MS);
}

// =================================================================
// HUVUDPROGRAM
// =================================================================
int main() {
    int sockfd, i2c_styr_fd, i2c_sens_fd; 
    struct sockaddr_in servaddr, cliaddr; 
    unsigned char buffer[BUFFER_SIZE]; 
    socklen_t len = sizeof(cliaddr); 

    uint8_t line_var = 0;
    uint8_t angle    = 0;
    uint8_t gyro1    = 0;
    uint8_t gyro2    = 0;
    
    uint8_t flags             = 0;
    uint8_t flags_korsning    = 0;
    uint8_t flags_ny_korsning = 0;

    init_karta();

    FILE *clr = fopen(VERIFY_LOG_FILE, "w");
    if (clr) fclose(clr);
    printf("--- PI CORE: DUAL I2C (0x10 & 0x12) + UDP ROUTER ---\n");

    // --- I2C: Motorstyrning (0x12) ---
    i2c_styr_fd = open(I2C_DEVICE, O_RDWR); 
    if (i2c_styr_fd >= 0) { 
        ioctl(i2c_styr_fd, I2C_SLAVE, STYRKOMM_ADDR); 
        if (write(i2c_styr_fd, NULL, 0) < 0) { 
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
    i2c_sens_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_sens_fd >= 0) {
        ioctl(i2c_sens_fd, I2C_SLAVE, SENSOR_ADDR);
        if (write(i2c_sens_fd, NULL, 0) < 0) {
            sim_sensor = true;
            printf("[SIM] Sensor Board (0x10) missing. Using time-based intersection simulation (%d ms).\n", SIM_SEGMENT_MS);
        } else {
            printf("Connected to Sensor Board (0x10)\n");
        }
    } else {
        sim_sensor = true;
        printf("[SIM] Could not open I2C for Sensor Board. Using time-based intersection simulation (%d ms).\n", SIM_SEGMENT_MS);
    }

    if (sim_sensor || sim_motor) {
        printf("\n*** RUNNING IN SIM MODE ***\n\n");
    }

    // --- UDP Socket ---
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { 
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }
    
    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&servaddr, 0, sizeof(servaddr)); 
    servaddr.sin_family      = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port        = htons(UDP_PORT); 

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) { 
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }
    printf("Listening for UDP on port %d...\n\n", UDP_PORT);

    // =============================================================
    // NON-BLOCKING MAIN LOOP (~500 Hz)
    // =============================================================
    while (1) {
        // ---------------------------------------------------------
        // 1. READ FROM SENSOR (0x10)
        // ---------------------------------------------------------
        unsigned char sensor_packet[PACKET_SIZE];
        if (!sim_sensor && i2c_sens_fd >= 0 && read(i2c_sens_fd, sensor_packet, PACKET_SIZE) == PACKET_SIZE) { 
            
            static unsigned char last_sensor_packet[PACKET_SIZE] = {0};
            if (memcmp(sensor_packet, last_sensor_packet, PACKET_SIZE) != 0) {
                log_sensor_data(sensor_packet);
                memcpy(last_sensor_packet, sensor_packet, PACKET_SIZE);
            }
            
            flags    = sensor_packet[0];
            line_var = sensor_packet[1];
            angle    = sensor_packet[2];
            gyro1    = sensor_packet[6];
            gyro2    = sensor_packet[7];

            flags_korsning = (flags & 0x0C) >> 2;
            
            if (!flags_ny_korsning) {
                flags_ny_korsning = (flags & 0x20) >> 4; 
            }
        }

        // ---------------------------------------------------------
        // 2. NETWORK PACKETS
        // ---------------------------------------------------------
        int n = recvfrom(sockfd, buffer, BUFFER_SIZE, MSG_DONTWAIT, (struct sockaddr *)&cliaddr, &len);
        
        if (n > 0) {
            gui_known = true;
        }

        // --- 0x05 Command Packet ---
        if (n == PACKET_SIZE && buffer[0] == 0x05 && buffer[7] == 0xFF) {
            unsigned char state  = buffer[1];
            unsigned char target = buffer[2];
            char action          = (char)buffer[3];

            if (state == 0x00 || state == 0x01) {
                if (action == 'f' && current_phase == PHASE_IDLE) {
                    start_autonomous_sequence(state);
                } else {
                    buffer[4] = line_var;
                    buffer[5] = gyro1;
                    buffer[6] = gyro2;
                    if (!sim_motor) write(i2c_styr_fd, buffer, PACKET_SIZE);
                    log_verification(buffer, action);
                    printf("-> Manual Command Forwarded: '%c'\n", action);
                }
            } else if (state == 0x02 || state == 0x03) {
                if (current_phase != PHASE_IDLE) {
                    printf("\n[!] MANUAL OVERRIDE DETECTED. Canceling Auto Route.\n");
                    current_phase        = PHASE_IDLE;
                    is_rotating          = false;
                    is_picking_up        = false;
                    current_action_index = 0;
                    korsning_aktiv       = 0;
                    aktivt_beslut        = 's';
                    nasta_beslut         = 's';
                }
                buffer[4] = line_var;
                buffer[5] = gyro1;
                buffer[6] = gyro2;
                if (!sim_motor) write(i2c_styr_fd, buffer, PACKET_SIZE);
                log_verification(buffer, action);
                printf("-> Manual Command Forwarded: '%c'\n", action);
            }
        }

        // --- 0x07 Item List Packet ---
        if (n >= 4 && buffer[0] == 0x07) {
            int num = buffer[1];
            int expected_len = 3 + 2 * num;
            if (num > 0 && num <= MAX_ITEMS && n == expected_len && buffer[n-1] == 0xFF) {
                item_count = 0;
                for (int i = 0; i < num; i++) {
                    uint8_t iu = buffer[2 + 2*i];
                    uint8_t iv = buffer[3 + 2*i];
                    if (iu < NODES && iv < NODES && vag[iu][iv]) {
                        item_list_u[item_count] = iu;
                        item_list_v[item_count] = iv;
                        item_count++;
                    } else {
                        printf("[!] Skipping invalid item edge (%d, %d)\n", iu, iv);
                    }
                }
                current_item_index = 0;
                printf("[ITEMS] Received %d valid item(s) from GUI\n", item_count);
            } else {
                printf("[!] Invalid item-list packet (n=%d, count=%d)\n", n, num);
            }
        }

        // ---------------------------------------------------------
        // 3. AUTONOMOUS STATE MACHINE
        // ---------------------------------------------------------
        if (current_phase != PHASE_IDLE) { 
            
            long long elapsed_in_state = current_time_ms() - action_timer_start;

            if (is_rotating) {
                if (elapsed_in_state >= 10500) { 
                    is_rotating   = false;
                    aktivt_beslut = 'f';
                    
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
                if (elapsed_in_state >= 10000 && aktivt_beslut == 's') {
                    aktivt_beslut = 'v';
                    if (current_item_index + 1 < item_count) {
                        nasta_beslut = 'f';
                    } else {
                        nasta_beslut = beslut_hem[0];
                    }
                    log_next_action = true;
                }
                else if (elapsed_in_state >= 20000) {
                    is_picking_up = false;
                    current_item_index++;

                    if (current_item_index < item_count) {
                        vara_u = item_list_u[current_item_index];
                        vara_v = item_list_v[current_item_index];
                        printf("\n-> Item %d/%d: edge %d <-> %d\n",
                               current_item_index + 1, item_count, vara_u, vara_v);
                        planera_nasta_vara();
                        planera_hem_fran_pickup();

                        current_phase        = PHASE_TO_ITEM;
                        current_action_index = 0;
                        aktivt_beslut_fn(current_action_index);

                        if (aktivt_beslut == 'e' || aktivt_beslut == 'o' || aktivt_beslut == 'u') {
                            is_rotating = true;
                            action_timer_start = current_time_ms();
                        } else if (sim_sensor) {
                            sim_segment_timer = current_time_ms();
                        }
                        printf("-> PHASE CHANGE: Driving to item %d/%d...\n",
                               current_item_index + 1, item_count);
                        log_next_action = true;
                    } else {
                        planera_hem_fran_pickup();
                        current_phase        = PHASE_TO_HOME;
                        current_action_index = 0;
                        aktivt_beslut_fn(current_action_index);

                        if (aktivt_beslut == 'e' || aktivt_beslut == 'o' || aktivt_beslut == 'u') {
                            is_rotating = true;
                            action_timer_start = current_time_ms();
                        } else if (sim_sensor) {
                            sim_segment_timer = current_time_ms();
                        }
                        printf("\n-> PHASE CHANGE: All %d items collected. Heading Home...\n", item_count);
                        log_next_action = true;
                    }
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
                    if ((flags_korsning == 2 || flags_korsning == 1) && !korsning_aktiv) {
                        intersection_triggered = true;
                        korsning_aktiv    = 1;
                        flags_ny_korsning = 0;
                    } else if (flags_korsning == 0) {
                        korsning_aktiv = 0;
                    }
                }

                if (intersection_triggered) {
                    current_action_index++;
                    aktivt_beslut_fn(current_action_index);
                    action_timer_start = current_time_ms();

                    // Uppdatera aktuell nod och riktning
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

                    if (aktivt_beslut == 'e' || aktivt_beslut == 'o' || aktivt_beslut == 'u') {
                        is_rotating = true;
                        log_next_action = true;
                    }
                    else if (aktivt_beslut == 'X') {
                        if (current_phase == PHASE_TO_ITEM) {
                            current_phase = PHASE_PICKUP;
                            aktivt_beslut = 's';
                            nasta_beslut  = 'v';
                            is_picking_up = true;
                            printf("\n-> Pickup item %d/%d...\n", current_item_index + 1, item_count);
                            log_next_action = true;
                        }
                        else if (current_phase == PHASE_TO_HOME) {
                            current_phase = PHASE_IDLE;
                            current_node  = START;
                            aktivt_beslut = 's';
                            nasta_beslut  = 's';
                            printf("\n=== AUTONOMOUS ROUTE COMPLETE ===\n\n");
                            
                            unsigned char stop_packet[PACKET_SIZE] = {
                                0x05, current_auto_state, 0x00, 's', 
                                line_var, gyro1, gyro2, 0xFF
                            };
                            if (!sim_motor) write(i2c_styr_fd, stop_packet, PACKET_SIZE);
                        }
                    }
                    else {
                        log_next_action = true;
                    }

                    if (sim_sensor && !is_rotating && !is_picking_up && aktivt_beslut != 'X') {
                        sim_segment_timer = current_time_ms();
                    }
                }
            }

            // --- SEND MOTOR COMMANDS ---
            if (current_phase != PHASE_IDLE && aktivt_beslut != 'X') {
                
                char skickat_kommando = aktivt_beslut;
                
                if (is_rotating && (current_time_ms() - action_timer_start < 500)) {
                    skickat_kommando = 's';
                }

                unsigned char auto_packet[PACKET_SIZE] = {
                    0x05,
                    current_auto_state,
                    (current_phase == PHASE_PICKUP && aktivt_beslut == 'v') ? 0x01 : 0x00,
                    skickat_kommando,
                    line_var, gyro1, gyro2, 0xFF
                };

                if (!sim_motor) write(i2c_styr_fd, auto_packet, PACKET_SIZE);
                
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

        // ---------------------------------------------------------
        // 4. TELEMETRY (10Hz)
        // ---------------------------------------------------------
        if (gui_known) {
            telemetry_counter++;
            if (telemetry_counter >= 50) {
                unsigned char telemetry_packet[(PACKET_SIZE+5)] = {
                    0x06,
                    (unsigned char)current_phase,
                    aktivt_beslut,
                    nasta_beslut,
                    line_var, gyro1, gyro2, flags,
                    current_node,
                    (unsigned char)current_item_index,
                    (unsigned char)item_count,
                    (unsigned char)current_dir,
                    0xFF
                };
                sendto(sockfd, telemetry_packet, (PACKET_SIZE+5), 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
                telemetry_counter = 0;
            }
        }
        
        // ---------------------------------------------------------
        // 5. TINY DELAY (2ms / 500Hz)
        // ---------------------------------------------------------
        usleep(2000); 
    }

    close(sockfd);
    if (i2c_styr_fd >= 0) close(i2c_styr_fd);
    if (i2c_sens_fd >= 0) close(i2c_sens_fd);
    return 0;
}