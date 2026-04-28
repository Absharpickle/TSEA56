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
#include <time.h>
#include <stdint.h> // Required for int16_t and uint8_t

// --- ALGORITHM DEFINITIONS ---
#define NODES 26
#define START 25 
#define NONE -1
#define STOP -1

// --- TELEMETRY DEFINITIONS ---
#define UDP_PORT 5001
#define BUFFER_SIZE 1024
#define I2C_DEVICE "/dev/i2c-1"
#define STYRKOMM_ADDR 0x12
#define SENSOR_ADDR 0x10    // NEW: Sensor I2C Address
#define PACKET_SIZE 8
#define VERIFY_LOG_FILE "verifikation_keys.txt"

// --- ALGORITHM GLOBALS ---
char nodriktningsmatris[NODES][NODES];
int  vag[NODES][NODES];
int  rutt_till_vara[NODES];
int  rutt_hem[NODES];
char beslut_till_vara[NODES];
char beslut_hem[NODES];
int  vara_u, vara_v; 

// --- STATE MACHINE GLOBALS ---
typedef enum {
    PHASE_IDLE = 0,
    PHASE_TO_ITEM,
    PHASE_PICKUP,
    PHASE_TO_HOME
} AutoPhase;

AutoPhase current_phase = PHASE_IDLE;
int current_action_index = 0;
unsigned char current_auto_state = 1;
bool log_next_action = false;

// --- LOOP TIMING GLOBALS ---
char aktivt_beslut = 's'; // Default to stop
int  loop_counter = 0;

// =================================================================
// 1. KARTA, HJÄLPFUNKTIONER & RUTTPLANERING
// =================================================================
void init_karta() {
    memset(vag, 0, sizeof(vag));
    memset(nodriktningsmatris, ' ', sizeof(nodriktningsmatris));
    for (int i = 0; i < 25; i++) {
        int rad = i / 5;
        int kol = i % 5;
        if (kol < 4) { vag[i][i+1] = 1; nodriktningsmatris[i][i+1] = 'e'; } 
        if (kol > 0) { vag[i][i-1] = 1; nodriktningsmatris[i][i-1] = 'w'; } 
        if (rad < 4) { vag[i][i+5] = 1; nodriktningsmatris[i][i+5] = 's'; } 
        if (rad > 0) { vag[i][i-5] = 1; nodriktningsmatris[i][i-5] = 'n'; } 
    }
    vag[START][0] = 1; 
    vag[0][START] = 1;
    nodriktningsmatris[START][0] = 's';
    nodriktningsmatris[0][START] = 'n';
}

char get_turn(char nu, char nasta) {
    if (nu == nasta) return 'f';
    if (nu == 'n' && nasta == 'e') return 'r';
    if (nu == 'e' && nasta == 's') return 'r';
    if (nu == 's' && nasta == 'w') return 'r';
    if (nu == 'w' && nasta == 'n') return 'r';
    if (nu == 'n' && nasta == 'w') return 'l';
    if (nu == 'w' && nasta == 's') return 'l';
    if (nu == 's' && nasta == 'e') return 'l';
    if (nu == 'e' && nasta == 'n') return 'l';
    return 'b';
}

char get_motsatt_dir(char nu) {
    if (nu == 's') return 'n';
    if (nu == 'n') return 's';
    if (nu == 'e') return 'w';
    if (nu == 'w') return 'e';
    return nu; 
}

void bygg_beslut(int rutt[], char start_dir, char beslut[]) {
    char dir = start_dir;
    int i = 0;
    while (rutt[i + 1] != STOP) {
        char nasta_dir = nodriktningsmatris[rutt[i]][rutt[i + 1]];
        beslut[i] = get_turn(dir, nasta_dir);
        dir = nasta_dir;
        i++;
    }
    beslut[i] = 'X'; 
    beslut[i+1] = '\0';
}

int hitta_rutt(int start, int mal, int rutt[], char start_dir) {
    int kostnad[NODES], foregaende[NODES];
    char riktning_in[NODES];
    bool besokt[NODES] = {false};

    for (int i = 0; i < NODES; i++) {
        kostnad[i] = 9999;
        foregaende[i] = NONE;
        rutt[i] = STOP;
    }

    kostnad[start] = 0;
    riktning_in[start] = start_dir;

    for (int i = 0; i < NODES; i++) {
        int u = -1;
        for (int j = 0; j < NODES; j++) {
            if (!besokt[j] && (u == -1 || kostnad[j] < kostnad[u])) u = j;
        }
        if (kostnad[u] == 9999 || u == mal) break;
        besokt[u] = true;

        for (int v = 0; v < NODES; v++) {
            if (vag[u][v] && !besokt[v]) {
                char nasta_dir = nodriktningsmatris[u][v];
                int straff = (riktning_in[u] != nasta_dir) ? 1 : 0;
                int ny_kostnad = kostnad[u] + 100 + straff;
                if (ny_kostnad < kostnad[v]) {
                    kostnad[v] = ny_kostnad;
                    foregaende[v] = u;
                    riktning_in[v] = nasta_dir;
                }
            }
        }
    }

    int temp[NODES], c = 0, nu = mal;
    while (nu != NONE) {
        temp[c++] = nu;
        nu = foregaende[nu];
    }
    for (int i = 0; i < c; i++) rutt[i] = temp[c - 1 - i];
    return kostnad[mal];
}

void planera_hela_resan(int nuvarande_nod, char nuvarande_dir) {
    int rutt_alt1[NODES], rutt_alt2[NODES];
    int kostnad1 = hitta_rutt(nuvarande_nod, vara_u, rutt_alt1, nuvarande_dir);
    int kostnad2 = hitta_rutt(nuvarande_nod, vara_v, rutt_alt2, nuvarande_dir);
    
    int ingang, utgang;
    if (kostnad1 <= kostnad2) {
        ingang = vara_u; utgang = vara_v;
        memcpy(rutt_till_vara, rutt_alt1, sizeof(rutt_alt1));
    } else {
        ingang = vara_v; utgang = vara_u;
        memcpy(rutt_till_vara, rutt_alt2, sizeof(rutt_alt2));
    }

    int i = 0;
    while (rutt_till_vara[i] != STOP) i++;
    rutt_till_vara[i] = utgang;
    rutt_till_vara[i+1] = STOP;

    bygg_beslut(rutt_till_vara, nuvarande_dir, beslut_till_vara);
    
    char dir_vid_vara = nodriktningsmatris[ingang][utgang]; 
    int cost_utgang = hitta_rutt(utgang, START, rutt_alt1, dir_vid_vara);
    int cost_ingang = hitta_rutt(ingang, START, rutt_alt2, dir_vid_vara) + 100; 

    if (cost_utgang <= cost_ingang) {
        memcpy(rutt_hem, rutt_alt1, sizeof(rutt_alt1));
        bygg_beslut(rutt_hem, dir_vid_vara, beslut_hem);
    } else {
        memcpy(rutt_hem, rutt_alt2, sizeof(rutt_alt2));
        char dir_efter_vanding = get_motsatt_dir(dir_vid_vara);
        bygg_beslut(rutt_hem, dir_efter_vanding, beslut_hem);
        int len = strlen(beslut_hem) + 1;
        memmove(&beslut_hem[1], &beslut_hem[0], len);
        beslut_hem[0] = 'b'; 
    }
}

// =================================================================
// 2. I2C, TELEMETRY & AUTO-INIT FUNKTIONER
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

void aktivt_beslut_fn(int index) {
    if (current_phase == PHASE_TO_ITEM) {
        aktivt_beslut = beslut_till_vara[index];
    } else if (current_phase == PHASE_PICKUP) {
        aktivt_beslut = 'v'; // Arm pickup action
    } else if (current_phase == PHASE_TO_HOME) {
        aktivt_beslut = beslut_hem[index];
    }
}

void start_autonomous_sequence(unsigned char state) {
    vara_u = 6; 
    vara_v = 7; 
    
    printf("\n=== CALCULATING AUTONOMOUS ROUTE ===\n");
    planera_hela_resan(START, 's');
    
    current_auto_state = state;
    current_phase = PHASE_TO_ITEM;
    current_action_index = 0;
    
    loop_counter = 0;
    aktivt_beslut_fn(current_action_index); 
    log_next_action = true; 
    
    printf("-> Route Calculated. Driving to item...\n");
}

// =================================================================
// 3. HUVUDPROGRAM (TRUE CONTINUOUS NON-BLOCKING LOOP)
// =================================================================
int main() {
    int sockfd, i2c_styr_fd, i2c_sens_fd;
    struct sockaddr_in servaddr, cliaddr;
    unsigned char buffer[BUFFER_SIZE];
    socklen_t len = sizeof(cliaddr);

    // Globals to hold the incoming sensor values
    uint8_t line_var = 0;
    uint8_t angle = 0;
    uint8_t gyro1 = 0;
    uint8_t gyro2 = 0;

    init_karta();

    FILE *clr = fopen(VERIFY_LOG_FILE, "w");
    if (clr) fclose(clr);
    printf("--- PI CORE: DUAL I2C (0x10 & 0x12) + UDP ROUTER ---\n");

    // SETUP I2C - STYRKOMM (0x12)
    i2c_styr_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_styr_fd >= 0) {
        ioctl(i2c_styr_fd, I2C_SLAVE, STYRKOMM_ADDR);
        if (write(i2c_styr_fd, NULL, 0) < 0) {
            printf("[WARNING] Motor Controller (0x12) missing. Running in Sim Mode.\n");
        } else {
            printf("Connected to Motor Controller (0x12)\n");
        }
    }

    // SETUP I2C - SENSOR (0x10)
    i2c_sens_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_sens_fd >= 0) {
        ioctl(i2c_sens_fd, I2C_SLAVE, SENSOR_ADDR);
        if (write(i2c_sens_fd, NULL, 0) < 0) {
            printf("[WARNING] Sensor Board (0x10) missing. Will send zeros.\n");
        } else {
            printf("Connected to Sensor Board (0x10)\n");
        }
    }

    // SETUP UDP
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(UDP_PORT);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }
    printf("Listening for UDP on port %d...\n\n", UDP_PORT);

    // NON-BLOCKING MAIN LOOP
    while (1) {
        // -------------------------------------------------------------
        // 1. READ FROM SENSOR (0x10)
        // -------------------------------------------------------------
        unsigned char sensor_packet[PACKET_SIZE];
        if (i2c_sens_fd >= 0 && read(i2c_sens_fd, sensor_packet, PACKET_SIZE) == PACKET_SIZE) {
            // Note: Assuming standard High-Byte first (Big Endian). 
            // If values are weird, swap [1] with [2] and [3] with [4].
            int16_t val1 = (int16_t)((sensor_packet[2] << 8) | sensor_packet[1]); //Fabian nils och adam bytte till little endian 1 till 2
            int16_t val2 = (int16_t)((sensor_packet[3] << 8) | sensor_packet[4]);
            
            //line_var = (uint8_t)((val1 + val2) / 2);
            line_var = sensor_packet[1];
            angle = sensor_packet[2];
            gyro1 = sensor_packet[6];
            gyro2 = sensor_packet[7];
        }

        // -------------------------------------------------------------
        // 2. CHECK FOR NETWORK PACKETS (INSTANTLY)
        // -------------------------------------------------------------
        int n = recvfrom(sockfd, buffer, BUFFER_SIZE, MSG_DONTWAIT, (struct sockaddr *)&cliaddr, &len);
        
        if (n == PACKET_SIZE && buffer[0] == 0x05 && buffer[7] == 0xFF) {
            unsigned char state = buffer[1];
            unsigned char target = buffer[2];
            char action = (char)buffer[3];

            if (state == 0x00 || state == 0x01) {
                // Start Auto Mode if 'f' is pressed
                if (action == 'f' && current_phase == PHASE_IDLE) {
                    start_autonomous_sequence(state);
                }
                else 
                {
                                    // Inject Sensor Data before forwarding!
                    buffer[4] = line_var;
                    buffer[5] = gyro1;
                    buffer[6] = gyro2;

                    write(i2c_styr_fd, buffer, PACKET_SIZE);
                    log_verification(buffer, action);
                    printf("-> Manual Command Forwarded: '%c'\n", action);
                }
            } else if (state == 0x02 || state == 0x03) {
                // MANUAL OVERRIDE
                if (current_phase != PHASE_IDLE) {
                    printf("\n[!] MANUAL OVERRIDE DETECTED. Canceling Auto Route.\n");
                    current_phase = PHASE_IDLE;
                }
                
                // Inject Sensor Data before forwarding!
                buffer[4] = line_var;
                buffer[5] = gyro1;
                buffer[6] = gyro2;

                write(i2c_styr_fd, buffer, PACKET_SIZE);
                log_verification(buffer, action);
                printf("-> Manual Command Forwarded: '%c'\n", action);
            }
        }

        // -------------------------------------------------------------
        // 3. AUTONOMOUS STATE MACHINE 
        // -------------------------------------------------------------
        if (current_phase != PHASE_IDLE) {
            
            loop_counter++;

            // 2500 loops * 2000 microseconds = 5,000,000us (Exactly 5 Seconds)
            if (loop_counter == 2500) {
                current_action_index++;
                aktivt_beslut_fn(current_action_index);
                
                log_next_action = true; 

                if (current_phase == PHASE_TO_ITEM && aktivt_beslut == 'X') {
                    current_phase = PHASE_PICKUP;
                    current_action_index = 0;
                    aktivt_beslut_fn(current_action_index);
                    printf("\n-> PHASE CHANGE: Picking up item...\n");
                } 
                else if (current_phase == PHASE_PICKUP) {
                    current_phase = PHASE_TO_HOME;
                    current_action_index = 0;
                    aktivt_beslut_fn(current_action_index);
                    printf("\n-> PHASE CHANGE: Heading Home...\n");
                } 
                else if (current_phase == PHASE_TO_HOME && aktivt_beslut == 'X') {
                    current_phase = PHASE_IDLE;
                    printf("\n=== AUTONOMOUS ROUTE COMPLETE ===\n\n");
                    
                    // Send a final stop command with active sensor data
                    unsigned char stop_packet[PACKET_SIZE] = {
                        0x05, current_auto_state, 0x00, 's', 
                        line_var, gyro1, gyro2, 0xFF
                    };
                    write(i2c_styr_fd, stop_packet, PACKET_SIZE);
                    log_verification(stop_packet, 's');
                }

                loop_counter = 0;
            }

            // BLAST THE CURRENT ACTION CONTINUOUSLY
            if (current_phase != PHASE_IDLE && aktivt_beslut != 'X') {
                
                unsigned char auto_packet[PACKET_SIZE] = {
                    0x05, 
                    current_auto_state, 
                    (current_phase == PHASE_PICKUP) ? 0x01 : 0x00, // Arm or Wheel
                    aktivt_beslut, 
                    line_var,  // Inject calculated line sensor
                    gyro1,     // Inject direct gyro 1
                    gyro2,     // Inject direct gyro 2
                    0xFF
                };

                // SEND TO MICROCONTROLLER EVERY LOOP ITERATION
                write(i2c_styr_fd, auto_packet, PACKET_SIZE);
                log_verification(auto_packet, auto_packet[3]);

                if (log_next_action) {
                    printf("Action updated to: '%c'\n", auto_packet[3]);
                    log_next_action = false;
                }
            }
        }

        // -------------------------------------------------------------
        // 4. TINY DELAY (2000 microseconds = 2 milliseconds / 500Hz)
        // -------------------------------------------------------------
        usleep(2000); 
    }

    close(sockfd);
    if (i2c_styr_fd >= 0) close(i2c_styr_fd);
    if (i2c_sens_fd >= 0) close(i2c_sens_fd);
    return 0;
}
