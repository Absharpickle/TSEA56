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
int telemetry_counter = 0;
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

// --- HINDERHANTERING (NY) ---
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
        pending_rotation_cmd = aktivt_beslut;
        aktivt_beslut = 's';
        action_timer_start = current_time_ms();
    } else if (sim_sensor) {
        sim_segment_timer = current_time_ms();
    }

    log_next_action = true;
    route_changed  = true;
    printf("-> Route Calculated. Driving to item 1/%d...\n", item_count);
    if (sim_sensor) printf("[SIM] Intersections will be triggered every %d ms\n", SIM_SEGMENT_MS);
}

void init_system(SystemPointers *sys) {
    init_karta();

    FILE *clr = fopen(VERIFY_LOG_FILE, "w");
    if (clr) fclose(clr);
    printf("--- PI CORE: DUAL I2C (0x10 & 0x12) + UDP ROUTER ---\n");

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

    sys->i2c_sens_fd = open(I2C_DEVICE, O_RDWR);
    if (sys->i2c_sens_fd >= 0) {
        ioctl(sys->i2c_sens_fd, I2C_SLAVE, SENSOR_ADDR);
        if (write(sys->i2c_sens_fd, NULL, 0) < 0) {
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

// NYTT: hinder-bool tillagd
void update_sensors(SystemPointers *sys, uint8_t *line_var_f, uint8_t *line_var_b, uint8_t *angle, uint8_t *gyro1, uint8_t *gyro2, uint8_t *flags, uint8_t *flags_korsning, uint8_t *flags_ny_korsning, bool *obstacle_detected) {
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
            *flags_ny_korsning = (*flags & 0x20) >> 4; // Notera: Du kan behöva ändra detta om sensorn skickar bit 5 för ny korsning istället.
        }
        
        // UPPDATERING: Bit 5 är 0x20. Hindersensor triggad
        *obstacle_detected = (*flags & 0x20) != 0;

        if (*flags_korsning == 1)      pickup_cmd = 'v';
        else if (*flags_korsning == 3) pickup_cmd = 'h';
    }
}

void process_network_packets(SystemPointers *sys, uint8_t line_var_f, uint8_t line_var_b, uint8_t gyro1, uint8_t gyro2) {
    unsigned char buffer[BUFFER_SIZE];
    int n = recvfrom(sys->sockfd, buffer, BUFFER_SIZE, MSG_DONTWAIT, (struct sockaddr *)&sys->cliaddr, &sys->cliaddr_len);
    if (n > 0) gui_known = true;

    CommandPacket cmd = parse_command_packet(buffer, n);
    if (cmd.valid) {
        if (cmd.state == 0x00 || cmd.state == 0x01) {
            if (cmd.action == 'f' && current_phase == PHASE_IDLE) {
                start_autonomous_sequence(cmd.state);
            } else {
                unsigned char fwd[PACKET_SIZE];
                build_motor_packet(fwd, cmd.state, false, cmd.action, line_var_f, line_var_b, gyro1, gyro2);
                fwd[2] = cmd.target; 
                if (!sim_motor) write(sys->i2c_styr_fd, fwd, PACKET_SIZE);
                log_verification(fwd, cmd.action);
                printf("-> Manual Command Forwarded: '%c'\n", cmd.action);
            }
        } else if (cmd.state == 0x02 || cmd.state == 0x03) {
            
            init_karta(); // <-- UPPDATERING: Glöm alla blockeringar vid manuell override!
            
            if (current_phase != PHASE_IDLE) {
                printf("\n[!] MANUAL OVERRIDE DETECTED. Map reset. Canceling Auto Route.\n");
                current_phase = PHASE_IDLE;
                is_rotating = false;
                is_picking_up = false;
                is_dropping = false;
                current_action_index = 0;
                korsning_aktiv = 0;
                aktivt_beslut = 's';
                nasta_beslut = 's';
                is_handling_obstacle = false; // Glöm ifall vi stannade pga hinder
            }
            unsigned char fwd[PACKET_SIZE];
            build_motor_packet(fwd, cmd.state, false, cmd.action, line_var_f, line_var_b, gyro1, gyro2);
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

void process_autonomous_state(SystemPointers *sys, uint8_t line_var_f, uint8_t line_var_b, uint8_t gyro1, uint8_t gyro2, uint8_t flags_korsning, uint8_t *flags_ny_korsning, bool obstacle_detected) {
    if (current_phase == PHASE_IDLE) return;
    
    long long elapsed_in_state = current_time_ms() - action_timer_start;

    // =================================================================
    // HINDERHANTERING (DELAY & OMKALKYLERING)
    // =================================================================
    if (is_handling_obstacle) {
        if (current_time_ms() - obstacle_timer_start >= 500) { // 0.5s pause
            is_handling_obstacle = false;
            aktivt_beslut = 'f'; // Kör framåt mot nästa korsning där nya rutten börjar
            log_next_action = true;
        } else {
            // Skicka ständigt 's' under pausen
            unsigned char stop_pkt[PACKET_SIZE];
            build_motor_packet(stop_pkt, current_auto_state, false, 's', line_var_f, line_var_b, gyro1, gyro2);
            if (!sim_motor) write(sys->i2c_styr_fd, stop_pkt, PACKET_SIZE);
            return; 
        }
    }

    // Hittade ett hinder? (och håller inte redan på med en annan åtgärd)
    if (obstacle_detected && !is_handling_obstacle && !is_rotating && !is_picking_up && !is_dropping && korsning_aktiv == 0) {
        int approaching_node = -1;
        int *current_route = (current_phase == PHASE_TO_ITEM) ? rutt_till_vara : rutt_hem;
        char *current_beslut = (current_phase == PHASE_TO_ITEM) ? beslut_till_vara : beslut_hem;

        if (current_action_index + 1 < NODES) {
            approaching_node = current_route[current_action_index + 1];
        }

        if (approaching_node >= 0 && approaching_node < 25) {
            int blocked_node = -1;
            
            // Hitta noden vi skulle åkt till EFTER approaching_node i samma riktning
            for (int i = 0; i < 25; i++) {
                if (vag[approaching_node][i] && nodriktningsmatris[approaching_node][i] == current_dir) {
                    blocked_node = i;
                    break;
                }
            }

            if (blocked_node != -1) {
                printf("\n[!] HINDER UPPTÄCKT! Tar bort väg %d <-> %d och räknar om rutt...\n", approaching_node, blocked_node);
                
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
                nasta_beslut = current_beslut[0];
                route_changed = true; 
                
                is_handling_obstacle = true;
                obstacle_timer_start = current_time_ms();
                
                unsigned char stop_pkt[PACKET_SIZE];
                build_motor_packet(stop_pkt, current_auto_state, false, 's', line_var_f, line_var_b, gyro1, gyro2);
                if (!sim_motor) write(sys->i2c_styr_fd, stop_pkt, PACKET_SIZE);
                
                return; // Bryt loopen direkt för denna frame
            }
        }
    }
    // =================================================================

    if (is_rotating) {
        unsigned char rot_pkt[PACKET_SIZE];
        build_motor_packet(rot_pkt, current_auto_state, false, pending_rotation_cmd, line_var_f, line_var_b, gyro1, gyro2);

        if (!sim_motor) {
            unsigned char ack_buf[PACKET_SIZE];
            if (read(sys->i2c_styr_fd, ack_buf, PACKET_SIZE) == PACKET_SIZE) {
                log_styr_response(ack_buf);
                StyrResponse resp = parse_styr_response(ack_buf, PACKET_SIZE);
                if (resp.valid && resp.action_done == 1) {
                    rotation_done = true;
                }
            }
            write(sys->i2c_styr_fd, rot_pkt, PACKET_SIZE);
        } else {
            if (elapsed_in_state > 2000) rotation_done = true;
        }

        if (rotation_done) {
            is_rotating = false;
            rotation_done = false;
            current_dir = get_turn(current_dir, pending_rotation_cmd);
            aktivt_beslut_fn(current_action_index);
            
            if (aktivt_beslut == 'X') {
                is_picking_up = true;
                action_timer_start = current_time_ms();
            } else if (aktivt_beslut == 'e' || aktivt_beslut == 'o') {
                is_rotating = true;
                pending_rotation_cmd = aktivt_beslut;
                aktivt_beslut = 's';
                action_timer_start = current_time_ms();
            } else {
                log_next_action = true;
            }
        }
        return;
    }

    if (is_picking_up) {
        // Kör din befintliga upphämtningslogik... (exkluderad detalj för korthet)
        // ...
        return; 
    }

    if (is_dropping) {
        // Kör din befintliga lämningslogik... (exkluderad detalj för korthet)
        // ...
        return;
    }

    if (*flags_ny_korsning == 1 || (sim_sensor && current_time_ms() - sim_segment_timer > SIM_SEGMENT_MS)) {
        if (korsning_aktiv == 0) {
            korsning_aktiv = 1;
            current_action_index++;

            if (current_phase == PHASE_TO_ITEM) {
                current_node = rutt_till_vara[current_action_index];
            } else if (current_phase == PHASE_TO_HOME) {
                current_node = rutt_hem[current_action_index];
            }

            aktivt_beslut_fn(current_action_index);

            if (aktivt_beslut == 'X') {
                is_picking_up = true;
                action_timer_start = current_time_ms();
            } else if (aktivt_beslut == 'e' || aktivt_beslut == 'o') {
                is_rotating = true;
                pending_rotation_cmd = aktivt_beslut;
                aktivt_beslut = 's';
                action_timer_start = current_time_ms();
            } else {
                log_next_action = true;
            }

            if (sim_sensor) sim_segment_timer = current_time_ms();
            *flags_ny_korsning = 0; 
        }
    } else {
        korsning_aktiv = 0;
    }

    if (loop_counter % 5 == 0) {
        unsigned char m_pkt[PACKET_SIZE];
        build_motor_packet(m_pkt, current_auto_state, false, aktivt_beslut, line_var_f, line_var_b, gyro1, gyro2);
        if (!sim_motor) write(sys->i2c_styr_fd, m_pkt, PACKET_SIZE);
        if (log_next_action) {
            log_verification(m_pkt, aktivt_beslut);
            log_next_action = false;
        }
    }
    loop_counter++;
}

void send_telemetry_and_routes(SystemPointers *sys, uint8_t line_var_f, uint8_t gyro1, uint8_t gyro2, uint8_t flags) {
    if (!gui_known) return;

    if (telemetry_counter++ % 25 == 0) {
        unsigned char tpkt[PACKET_SIZE + 6];
        build_telemetry_packet(tpkt, (uint8_t)current_phase, aktivt_beslut, nasta_beslut, 
                               line_var_f, gyro1, gyro2, flags, current_node, 
                               current_item_index, item_count, current_dir, action_done);
        sendto(sys->sockfd, tpkt, PACKET_SIZE + 6, 0, (struct sockaddr *)&sys->cliaddr, sys->cliaddr_len);
    }

    if (route_changed) {
        route_changed = false;
        int *rutt = (current_phase == PHASE_TO_ITEM) ? rutt_till_vara : rutt_hem;
        unsigned char rpkt[NODES + 3];
        int rpkt_len = build_route_packet(rpkt, rutt, NODES);
        sendto(sys->sockfd, rpkt, rpkt_len, 0, (struct sockaddr *)&sys->cliaddr, sys->cliaddr_len);
    }
}

int main() {
    SystemPointers sys;
    uint8_t line_var_f = 0, line_var_b = 0, angle = 0, gyro1 = 0, gyro2 = 0;
    uint8_t flags = 0, flags_korsning = 0, flags_ny_korsning = 0;
    bool obstacle_detected = false; // Lades till här!

    init_system(&sys);

    if (sim_sensor || sim_motor) {
        printf("\n*** RUNNING IN SIM MODE ***\n\n");
    }

    while (1) {
        update_sensors(&sys, &line_var_f, &line_var_b, &angle, &gyro1, &gyro2, &flags, &flags_korsning, &flags_ny_korsning, &obstacle_detected);
        process_network_packets(&sys, line_var_f, line_var_b, gyro1, gyro2);
        process_autonomous_state(&sys, line_var_f, line_var_b, gyro1, gyro2, flags_korsning, &flags_ny_korsning, obstacle_detected);
        
        send_telemetry_and_routes(&sys, line_var_f, gyro1, gyro2, flags);
        
        usleep(25000); 
    }
    return 0;
}