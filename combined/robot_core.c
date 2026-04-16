#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <time.h>

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
time_t last_action_time = 0;
unsigned char current_auto_state = 1;
bool log_next_action = false;

// =================================================================
// 1. KARTA OCH HJÄLPFUNKTIONER (Från algoritm.c)
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

void start_autonomous_sequence(unsigned char state) {
    vara_u = 6; 
    vara_v = 7; 
    
    printf("\n=== CALCULATING AUTONOMOUS ROUTE ===\n");
    planera_hela_resan(START, 's');
    
    // Initialize the State Machine
    current_auto_state = state;
    current_phase = PHASE_TO_ITEM;
    current_action_index = 0;
    last_action_time = time(NULL);
    log_next_action = true; // Force the very first action to log
    
    printf("-> Route Calculated. Driving to item...\n");
}

// =================================================================
// 3. HUVUDPROGRAM (NON-BLOCKING EVENT LOOP)
// =================================================================
int main() {
    int sockfd, i2c_fd;
    struct sockaddr_in servaddr, cliaddr;
    unsigned char buffer[BUFFER_SIZE];
    socklen_t len = sizeof(cliaddr);
    fd_set readfds;
    struct timeval tv;

    init_karta();

    FILE *clr = fopen(VERIFY_LOG_FILE, "w");
    if (clr) fclose(clr);
    printf("--- PI CORE: CONTINUOUS I2C + NON-BLOCKING UDP ---\n");

    // SETUP I2C
    i2c_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd < 0) {
        printf("[WARNING] Failed to open I2C bus (/dev/i2c-1).\n");
        printf("          -> Running in Network-Only (Simulation) mode.\n");
    } else {
        if (ioctl(i2c_fd, I2C_SLAVE, STYRKOMM_ADDR) < 0) {
            printf("[WARNING] Failed to configure I2C address 0x12.\n");
            printf("          -> Running in Network-Only (Simulation) mode.\n");
        } else {
            // THE PHYSICAL PROBE: Try to write 0 bytes
            if (write(i2c_fd, NULL, 0) < 0) {
                printf("[WARNING] Microcontroller not found at 0x12!\n");
                printf("          Check your physical SDA, SCL, and GND connections.\n");
                printf("          -> Running in Network-Only (Simulation) mode.\n");
            } else {
                printf("Successfully connected AND verified physical I2C address 0x12\n");
            }
        }
    }

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
        // Setup select() to wait a maximum of 50ms for a network packet
        FD_ZERO(&readfds);
        FD_SET(sockfd, &readfds);
        tv.tv_sec = 0;
        tv.tv_usec = 50000; // 50ms (Creates a 20Hz Loop)

        int activity = select(sockfd + 1, &readfds, NULL, NULL, &tv);

        // --- NETWORK EVENT HANDLING ---
        if (activity > 0 && FD_ISSET(sockfd, &readfds)) {
            int n = recvfrom(sockfd, buffer, BUFFER_SIZE, MSG_WAITALL, (struct sockaddr *)&cliaddr, &len);
            
            if (n == PACKET_SIZE && buffer[0] == 0x05 && buffer[7] == 0xFF) {
                unsigned char state = buffer[1];
                unsigned char target = buffer[2];
                char action = (char)buffer[3];

                if (state == 1 || state == 2) {
                    if (action == 'f' && current_phase == PHASE_IDLE) {
                        start_autonomous_sequence(state);
                    }
                } else if (state == 3 || state == 4) {
                    // MANUAL OVERRIDE: If a manual command arrives, kill auto route
                    if (current_phase != PHASE_IDLE) {
                        printf("\n[!] MANUAL OVERRIDE DETECTED. Canceling Auto Route.\n");
                        current_phase = PHASE_IDLE;
                    }
                    write(i2c_fd, buffer, PACKET_SIZE);
                    log_verification(buffer, action);
                }
            }
        }

        // --- AUTONOMOUS STATE MACHINE (Runs Continuously) ---
        if (current_phase != PHASE_IDLE) {
            time_t now = time(NULL);

            // 1. Check if 5 seconds have passed to advance the route
            if (now - last_action_time >= 5) {
                last_action_time = now;
                current_action_index++;
                log_next_action = true; // Tell the system to log this new change

                if (current_phase == PHASE_TO_ITEM && beslut_till_vara[current_action_index] == 'X') {
                    current_phase = PHASE_PICKUP;
                    current_action_index = 0;
                    printf("\n-> PHASE CHANGE: Picking up item...\n");
                } 
                else if (current_phase == PHASE_PICKUP) {
                    current_phase = PHASE_TO_HOME;
                    current_action_index = 0;
                    printf("\n-> PHASE CHANGE: Heading Home...\n");
                } 
                else if (current_phase == PHASE_TO_HOME && beslut_hem[current_action_index] == 'X') {
                    current_phase = PHASE_IDLE;
                    printf("\n=== AUTONOMOUS ROUTE COMPLETE ===\n\n");
                }
            }

            // 2. Continously blast the current action down I2C
            if (current_phase != PHASE_IDLE) {
                unsigned char auto_packet[PACKET_SIZE] = {0x05, current_auto_state, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF};

                if (current_phase == PHASE_TO_ITEM) {
                    auto_packet[3] = beslut_till_vara[current_action_index];
                } else if (current_phase == PHASE_PICKUP) {
                    auto_packet[2] = 0x01; // Change target to Arm
                    auto_packet[3] = 'v';
                } else if (current_phase == PHASE_TO_HOME) {
                    auto_packet[3] = beslut_hem[current_action_index];
                }

                // Blast it down the wire!
                write(i2c_fd, auto_packet, PACKET_SIZE);
            }
        }
    }

    close(sockfd);
    close(i2c_fd);
    return 0;
}