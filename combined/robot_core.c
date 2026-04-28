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
#include <stdint.h> 

// --- ALGORITHM DEFINITIONS ---
#define NODES 26 // 5x5 samt en start/slutnod
#define START 25 // Start/slut på nod 25
#define NONE -1 // Betyder att det inte finns en föregående nod
#define STOP -1 // Stoppvillkor för ruttarray

// --- TELEMETRY DEFINITIONS ---
#define UDP_PORT 5001 // Porten som används för kommunikation med persondatorn
#define BUFFER_SIZE 1024 // Storleken på bufferten
#define I2C_DEVICE "/dev/i2c-1" // Filnamn för i2c-bussen
#define STYRKOMM_ADDR 0x12
#define SENSOR_ADDR 0x10
#define PACKET_SIZE 8 // Paketstorleken för kommunikationen inom systemet (i2c)
#define VERIFY_LOG_FILE "verifikation_keys.txt" // Loggfil

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

bool is_rotating = false;
bool is_picking_up = false;
time_t action_timer_start = 0; 
uint8_t korsning_aktiv = 0; 

// --- TELEMETRY GLOBALS FÖR GUI ---
bool gui_known = false; 
int telemetry_counter = 0; 

// --- LOOP TIMING GLOBALS ---
char nasta_beslut  = 's'; 
char aktivt_beslut = 's'; 
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
    if (nu == 'n' && nasta == 'e') return 'e'; 
    if (nu == 'e' && nasta == 's') return 'e';
    if (nu == 's' && nasta == 'w') return 'e';
    if (nu == 'w' && nasta == 'n') return 'e';
    if (nu == 'n' && nasta == 'w') return 'o'; 
    if (nu == 'w' && nasta == 's') return 'o';
    if (nu == 's' && nasta == 'e') return 'o';
    if (nu == 'e' && nasta == 'n') return 'o';
    return 'u'; 
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
    int kostnad[NODES]; 
    int foregaende[NODES]; 
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
        beslut_hem[0] = 'u'; 
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

void aktivt_beslut_fn(int index) {
    if (current_phase == PHASE_TO_ITEM) {
        nasta_beslut = beslut_till_vara[index];
    } else if (current_phase == PHASE_PICKUP) {
        nasta_beslut = 'v'; 
    } else if (current_phase == PHASE_TO_HOME) {
        nasta_beslut = beslut_hem[index];
    }
}

void start_autonomous_sequence(unsigned char state) {
    vara_u = 10; 
    vara_v = 11; 
    
    printf("\n=== CALCULATING AUTONOMOUS ROUTE ===\n");
    planera_hela_resan(START, 's'); 
    
    current_auto_state = state; 
    current_phase = PHASE_TO_ITEM; 
    current_action_index = 0; 
    korsning_aktiv = 0; 
    
    loop_counter = 0; 
    aktivt_beslut_fn(current_action_index); 
    
    if (nasta_beslut == 'e' || nasta_beslut == 'o' || nasta_beslut == 'u') {
        aktivt_beslut = nasta_beslut; 
        is_rotating = true;
        action_timer_start = time(NULL); 
    } else {
        aktivt_beslut = 'f';
    }

    log_next_action = true; 
    printf("-> Route Calculated. Driving to item...\n");
}

// =================================================================
// 3. HUVUDPROGRAM
// =================================================================
int main() {
    int sockfd, i2c_styr_fd, i2c_sens_fd; 
    struct sockaddr_in servaddr, cliaddr; 
    unsigned char buffer[BUFFER_SIZE]; 
    socklen_t len = sizeof(cliaddr); 

    uint8_t line_var = 0;
    uint8_t angle = 0;
    uint8_t gyro1 = 0;
    uint8_t gyro2 = 0;
    
    uint8_t flags = 0;
    uint8_t flags_korsning = 0;
    uint8_t flags_ny_korsning = 0;

    init_karta(); 

    FILE *clr = fopen(VERIFY_LOG_FILE, "w");
    if (clr) fclose(clr);
    printf("--- PI CORE: DUAL I2C (0x10 & 0x12) + UDP ROUTER ---\n");

    i2c_styr_fd = open(I2C_DEVICE, O_RDWR); 
    if (i2c_styr_fd >= 0) { 
        ioctl(i2c_styr_fd, I2C_SLAVE, STYRKOMM_ADDR); 
        if (write(i2c_styr_fd, NULL, 0) < 0) { 
            printf("[WARNING] Motor Controller (0x12) missing. Running in Sim Mode.\n"); 
        } else {
            printf("Connected to Motor Controller (0x12)\n"); 
        }
    }

    i2c_sens_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_sens_fd >= 0) {
        ioctl(i2c_sens_fd, I2C_SLAVE, SENSOR_ADDR);
        if (write(i2c_sens_fd, NULL, 0) < 0) {
            printf("[WARNING] Sensor Board (0x10) missing. Will send zeros.\n");
        } else {
            printf("Connected to Sensor Board (0x10)\n");
        }
    }

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { 
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }
    
    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

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
           
            static unsigned char last_sensor_packet[PACKET_SIZE] = {0};
            if (memcmp(sensor_packet, last_sensor_packet, PACKET_SIZE) != 0) {
                log_sensor_data(sensor_packet);
                memcpy(last_sensor_packet, sensor_packet, PACKET_SIZE);
            }
            
            flags = sensor_packet[0];
            line_var = sensor_packet[1];
            angle = sensor_packet[2];
            gyro1 = sensor_packet[6];
            gyro2 = sensor_packet[7];

            flags_korsning = (flags & 0x0C) >> 2; 
            
            if (!flags_ny_korsning) {
                flags_ny_korsning = (flags & 0x20) >> 4; 
            }
        }

        // -------------------------------------------------------------
        // 2. CHECK FOR NETWORK PACKETS (INSTANTLY)
        // -------------------------------------------------------------
        int n = recvfrom(sockfd, buffer, BUFFER_SIZE, MSG_DONTWAIT, (struct sockaddr *)&cliaddr, &len);
        
        if (n > 0) {
            gui_known = true; 
        }

        if (n == PACKET_SIZE && buffer[0] == 0x05 && buffer[7] == 0xFF) {
            unsigned char state = buffer[1];
            unsigned char target = buffer[2];
            char action = (char)buffer[3];

            if (state == 0x00 || state == 0x01) {
                if (action == 'f' && current_phase == PHASE_IDLE) {
                    start_autonomous_sequence(state);
                } else {
                    buffer[4] = line_var;
                    buffer[5] = gyro1;
                    buffer[6] = gyro2;

                    write(i2c_styr_fd, buffer, PACKET_SIZE);
                    log_verification(buffer, action);
                    printf("-> Manual Command Forwarded: '%c'\n", action);
                }
            } else if (state == 0x02 || state == 0x03) {
                if (current_phase != PHASE_IDLE) {
                    printf("\n[!] MANUAL OVERRIDE DETECTED. Canceling Auto Route.\n");
                    current_phase = PHASE_IDLE;
                    is_rotating = false;
                    is_picking_up = false;
                    current_action_index = 0; 
                    korsning_aktiv = 0;       
                    aktivt_beslut = 's';      
                }
                
                buffer[4] = line_var;
                buffer[5] = gyro1;
                buffer[6] = gyro2;

                write(i2c_styr_fd, buffer, PACKET_SIZE);
                log_verification(buffer, action);
                printf("-> Manual Command Forwarded: '%c'\n", action);
            }
        }

        // -------------------------------------------------------------
        // 3. AUTONOMOUS STATE MACHINE (INTERSECTION-BASED TRIGGERS)
        // -------------------------------------------------------------
        if (current_phase != PHASE_IDLE) { 
            
            time_t elapsed_in_state = time(NULL) - action_timer_start;

            if (is_rotating) {
                if (elapsed_in_state >= 3) { 
                    is_rotating = false;
                    aktivt_beslut = 'f';
                    log_next_action = true;
                }
            } 
            else if (is_picking_up) {
                if (elapsed_in_state >= 2 && aktivt_beslut == 's') {
                    aktivt_beslut = 'v';
                    log_next_action = true;
                }
                else if (elapsed_in_state >= 5) {
                    is_picking_up = false;
                    current_phase = PHASE_TO_HOME;
                    current_action_index = 0; 
                    aktivt_beslut_fn(current_action_index); 
                    
                    if (nasta_beslut == 'e' || nasta_beslut == 'o' || nasta_beslut == 'u') {
                        aktivt_beslut = nasta_beslut; 
                        is_rotating = true;
                        action_timer_start = time(NULL);
                    } else {
                        aktivt_beslut = 'f';
                    }
                    printf("\n-> PHASE CHANGE: Heading Home...\n");
                    log_next_action = true;
                }
            } 
            else {
                // NORMAL KÖRNING: Vi väntar på flaggan 'flags_korsning' från sensorn
                // flags_korsning == 2 (korsning) ELLER flags_korsning == 1 (pickup)
                if ((flags_korsning == 2 || flags_korsning == 1) && !korsning_aktiv) {
                    korsning_aktiv = 1;    
                    flags_ny_korsning = 0; 
                    
                    current_action_index++;
                    aktivt_beslut_fn(current_action_index); 
                    action_timer_start = time(NULL); 

                    if (nasta_beslut == 'e' || nasta_beslut == 'o' || nasta_beslut == 'u') {
                        unsigned char stop_packet[PACKET_SIZE] = {
                            0x05, current_auto_state, 0x00, 's', 
                            line_var, gyro1, gyro2, 0xFF
                        };
                        write(i2c_styr_fd, stop_packet, PACKET_SIZE);
                        
                        aktivt_beslut = nasta_beslut; 
                        is_rotating = true;
                        log_next_action = true;
                    }
                    else if (nasta_beslut == 'X') {
                        if (current_phase == PHASE_TO_ITEM) {
                            current_phase = PHASE_PICKUP; 
                            aktivt_beslut = 's';
                            is_picking_up = true;
                            printf("\n-> PHASE CHANGE: Stopping before pickup...\n");
                            log_next_action = true;
                        }
                        else if (current_phase == PHASE_TO_HOME) {
                            current_phase = PHASE_IDLE;
                            aktivt_beslut = 's';
                            printf("\n=== AUTONOMOUS ROUTE COMPLETE ===\n\n");
                            
                            unsigned char stop_packet[PACKET_SIZE] = {
                                0x05, current_auto_state, 0x00, 's', 
                                line_var, gyro1, gyro2, 0xFF
                            };
                            write(i2c_styr_fd, stop_packet, PACKET_SIZE);
                        }
                    }
                    else {
                        aktivt_beslut = 'f';
                        log_next_action = true;
                    }
                } 
                else if (flags_korsning == 0) {
                    korsning_aktiv = 0; // Vi har rullat helt av korsningen/markeringen
                }
            }

            if (current_phase != PHASE_IDLE && aktivt_beslut != 'X') {
                
                unsigned char auto_packet[PACKET_SIZE] = {
                    0x05, 
                    current_auto_state, 
                    (current_phase == PHASE_PICKUP && aktivt_beslut == 'v') ? 0x01 : 0x00, 
                    aktivt_beslut, 
                    line_var,  
                    gyro1,     
                    gyro2,     
                    0xFF
                };

                write(i2c_styr_fd, auto_packet, PACKET_SIZE);
                
                if (log_next_action) {
                    printf("Action updated to: '%c' (Index: %d)\n", auto_packet[3], current_action_index);
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

        // -------------------------------------------------------------
        // 4. SKICKA TELEMETRI TILLBAKA TILL GUI (10Hz)
        // -------------------------------------------------------------
        if (gui_known) {
            telemetry_counter++;
            if (telemetry_counter >= 50) { 
                unsigned char telemetry_packet[(PACKET_SIZE+1)] = {
                    0x06,                         // 0x06 identifierar paketet som telemetri
                    (unsigned char)current_phase, // Aktuell fas
                    aktivt_beslut,                // Vad vi skickar till motorerna
                    nasta_beslut,                 // Nästa beslut som ska skickas till motorerna
                    line_var,                     // Sensordata: Linje
                    gyro1,                        // Sensordata: Gyro 1
                    gyro2,                        // Sensordata: Gyro 2
                    flags,                        // Sensordata: Flaggor (bitmaskade)
                    0xFF                          // Footer
                };
                sendto(sockfd, telemetry_packet, (PACKET_SIZE+1), 0, (struct sockaddr *)&cliaddr, len);
                telemetry_counter = 0;
            }
        }
        
        // -------------------------------------------------------------
        // 5. TINY DELAY (2000 microseconds = 2 milliseconds / 500Hz)
        // -------------------------------------------------------------
        usleep(2000); 
    }

    close(sockfd);
    if (i2c_styr_fd >= 0) close(i2c_styr_fd);
    if (i2c_sens_fd >= 0) close(i2c_sens_fd);
    return 0;
}