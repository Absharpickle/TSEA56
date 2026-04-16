#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/ioctl.h>
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
// 2. I2C & TELEMETRY FUNKTIONER
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

// Denna funktion ersätter 'simulator()'
void run_autonomous_sequence(int i2c_fd, unsigned char state) {
    vara_u = 6; 
    vara_v = 7; 
    char start_dir = 's'; 
    
    printf("\n=== STARTAR AUTONOM RUTT ===\n");
    planera_hela_resan(START, start_dir);

    unsigned char auto_packet[8] = {0x05, state, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF};

    // 1. RESAN TILL VARAN
    printf("Resa till vara: \n");
    for(int i = 0; i < NODES; i++) {
        if (beslut_till_vara[i] == 'X') break;
        
        auto_packet[3] = beslut_till_vara[i]; // Insert action (f, r, l, b)
        write(i2c_fd, auto_packet, PACKET_SIZE);
        log_verification(auto_packet, beslut_till_vara[i]);
        
        printf("-> Skickade auto-beslut: %c\n", beslut_till_vara[i]);
        sleep(5); 
    }
    
    // 2. PLOCKA UPP VARA (Simulerar armkommando)
    printf("[STOPP] Plockar upp vara...\n");
    auto_packet[2] = 0x01; // Byt target till arm
    auto_packet[3] = 'v';  // Plocka
    write(i2c_fd, auto_packet, PACKET_SIZE);
    sleep(5);
    auto_packet[2] = 0x00; // Tillbaka till wheel

    // 3. RESAN HEM
    printf("Resa hem: \n");
    for(int i = 0; i < NODES; i++) {
        if (beslut_hem[i] == 'X') break;
        
        auto_packet[3] = beslut_hem[i];
        write(i2c_fd, auto_packet, PACKET_SIZE);
        log_verification(auto_packet, beslut_hem[i]);
        
        printf("-> Skickade auto-beslut: %c\n", beslut_hem[i]);
        sleep(5);
    }
    printf("=== AUTONOM RUTT KLAR ===\n\n");
}

// =================================================================
// 3. HUVUDPROGRAM (EVENT LOOP)
// =================================================================
int main() {
    int sockfd, i2c_fd;
    struct sockaddr_in servaddr, cliaddr;
    unsigned char buffer[BUFFER_SIZE];
    socklen_t len = sizeof(cliaddr);

    // Initialisera karta
    init_karta();

    // Rensa logg
    FILE *clr = fopen(VERIFY_LOG_FILE, "w");
    if (clr) fclose(clr);
    printf("--- PI CORE: UDP -> I2C + AUTONOMOUS ---\n");

    // SETUP I2C
    i2c_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd < 0 || ioctl(i2c_fd, I2C_SLAVE, STYRKOMM_ADDR) < 0) {
        perror("Failed to connect to I2C");
        exit(EXIT_FAILURE);
    }
    printf("I2C Connected (0x12)\n");

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

    // MAIN LOOP
    while (1) {
        int n = recvfrom(sockfd, buffer, BUFFER_SIZE, MSG_WAITALL, (struct sockaddr *)&cliaddr, &len);
        if (n < 0) continue;

        if (n == PACKET_SIZE && buffer[0] == 0x05 && buffer[7] == 0xFF) {
            unsigned char state = buffer[1];
            unsigned char target = buffer[2];
            char action = (char)buffer[3];

            printf("Packet (State: %d, Target: %d, Action: %c)\n", state, target, action);

            // --- ROUTING LOGIC ---
            if (state == 1 || state == 2) {
                // AUTONOMOUS MODE
                // Vi använder 'f' (Pil Upp) för att starta sekvensen!
                if (action == 'f') {
                    run_autonomous_sequence(i2c_fd, state);
                } else {
                    printf("I Auto mode. Tryck Upp ('f') för att starta rutten.\n");
                }
            } else {
                // MANUAL MODE (State 3 or 4)
                ssize_t written = write(i2c_fd, buffer, PACKET_SIZE);
                if (written == PACKET_SIZE) {
                    log_verification(buffer, action);
                    printf("-> I2C Manual Forwarding: %c\n", action);
                }
            }
        }
    }

    close(sockfd);
    close(i2c_fd);
    return 0;
}