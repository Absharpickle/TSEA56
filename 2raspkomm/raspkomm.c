#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <termios.h>
#include <sys/select.h>

// =================================================================
// 1. TERMINAL-HANTERING
// =================================================================
struct termios orig_termios;

void disable_raw_mode() { 
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios); 
}

void enable_raw_mode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(disable_raw_mode); 
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON); 
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

int kbhit() {
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

// =================================================================
// 2. LOGGFUNKTIONER (Ny CSV-logg för grafer!)
// =================================================================
long long start_time_ms = 0;

long long get_current_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)(tv.tv_sec) * 1000 + (tv.tv_usec) / 1000;
}

void init_csv_log() {
    FILE *f = fopen("robot_data.csv", "w"); // 'w' skriver över gammal fil vid varje ny start
    if (f != NULL) {
        fprintf(f, "Tid_ms,Lage,Linjefel,Gyro,Action\n");
        fclose(f);
    }
    start_time_ms = get_current_time_ms();
}

void log_data_csv(uint8_t state, int16_t linjefel, int16_t gyro, char action) {
    FILE *f = fopen("robot_data.csv", "a"); // 'a' lägger till på en ny rad
    if (f == NULL) return;

    long long relative_time = get_current_time_ms() - start_time_ms;
    
    // Format: Tid_ms, Lage (1=Auto, 2=Manuell), Linjefel, Gyro, Kommando
    fprintf(f, "%lld,%d,%d,%d,%c\n", relative_time, state, linjefel, gyro, action);
    fclose(f);
}

void log_verifikation(uint8_t* sent, uint8_t* received) {
    FILE *f = fopen("verifikation.txt", "a");
    if (f == NULL) return;
    fprintf(f, "SKICKAT (0x12): ");
    for(int i = 0; i < 8; i++) fprintf(f, "%02X ", sent[i]);
    fprintf(f, "\nECHO    (0x12): ");
    for(int i = 0; i < 8; i++) fprintf(f, "%02X ", received[i]);

    if (memcmp(sent, received, 8) == 0) fprintf(f, " | MATCH ✔️\n\n");
    else fprintf(f, " | FEL ❌\n\n");
    fclose(f);
}

// =================================================================
// 3. DIJKSTRA & RUTTPLANERING
// =================================================================
#define NODES 27
#define START 25
#define END 26 
#define INF -1
#define WHITE 0
#define GRAY 1
#define BLACK 2
#define NONE -1
#define STOP -1

char nodriktningsmatris[NODES][NODES];
int vag[NODES][NODES];
int malrutt[NODES], slutrutt[NODES];
char malbeslut[NODES], slutbeslut[NODES];

void init_karta(){
    for (int i = 0; i < NODES; i++){
        for(int j = 0; j < NODES; j++){
            vag[i][j] = 0;
            nodriktningsmatris[i][j] = ' ';
        }
    }
    for (int i = 0; i < 25; i++){
        int rad = i / 5, kol = i % 5;
        if (kol < 4) { vag[i][i + 1] = 1; nodriktningsmatris[i][i + 1] = 'e'; }
        if (kol > 0) { vag[i][i - 1] = 1; nodriktningsmatris[i][i - 1] = 'w'; }
        if (rad < 4) { vag[i][i + 5] = 1; nodriktningsmatris[i][i + 5] = 's'; }
        if (rad > 0) { vag[i][i - 5] = 1; nodriktningsmatris[i][i - 5] = 'n'; }
    }
    vag[START][0] = 1; nodriktningsmatris[START][0] = 's';
    vag[0][START] = 1; nodriktningsmatris[0][START] = 'n';
    vag[24][END] = 1;  nodriktningsmatris[24][END] = 's';
    vag[END][24] = 1;  nodriktningsmatris[END][24] = 'n';
}

void optimeringsfunktion(int startnod, int slutnod, int rutt_array[]){
    int color[NODES], distance[NODES], prev[NODES], queue[NODES];
    int first = 0, last = 0;
    for (int u = 0; u < NODES; u++){ color[u] = WHITE; distance[u] = INF; prev[u] = NONE; rutt_array[u] = STOP; }
    color[startnod] = GRAY; distance[startnod] = 0; queue[last++] = startnod;
    while (first < last) {
        int u = queue[first++];
        if (u == slutnod) break;
        for (int v = 0; v < NODES; v++){
            if (vag[u][v] == 1 && color[v] == WHITE){
                color[v] = GRAY; distance[v] = distance[u] + 1; prev[v] = u; queue[last++] = v;
            }
        }
        color[u] = BLACK;
    }
    int temp_rutt[NODES], current = slutnod, count = 0;
    while (current != NONE){ temp_rutt[count++] = current; current = prev[current]; }
    for (int i = 0; i < count; i++){ rutt_array[i] = temp_rutt[count - 1 - i]; }
}

int rakna_langd(int rutt_array[]) { int langd = 0; while (rutt_array[langd] != STOP) langd++; return langd; }

int rakna_svangar(int rutt_array[], char start_riktning) {
    char nuvarande_riktning = start_riktning;
    int antal_svangar = 0;
    for (int i = 0; rutt_array[i + 1] != STOP; i++) {
        char mal_riktning = nodriktningsmatris[rutt_array[i]][rutt_array[i + 1]];
        if (nuvarande_riktning != mal_riktning) antal_svangar++;
        nuvarande_riktning = mal_riktning;
    }
    return antal_svangar;
}

void berakna_rutter(int vag_u, int vag_v, char start_riktning) {
    int rutt_u[NODES], rutt_v[NODES];
    optimeringsfunktion(START, vag_u, rutt_u); optimeringsfunktion(START, vag_v, rutt_v);
    
    int langd_u = rakna_langd(rutt_u); int langd_v = rakna_langd(rutt_v);
    int ingangsnod;
    if (langd_u < langd_v) ingangsnod = vag_u;
    else if (langd_v < langd_u) ingangsnod = vag_v;
    else ingangsnod = (rakna_svangar(rutt_u, start_riktning) <= rakna_svangar(rutt_v, start_riktning)) ? vag_u : vag_v;
    
    int *vald_rutt = (ingangsnod == vag_u) ? rutt_u : rutt_v;
    for (int i = 0; i < NODES; i++) malrutt[i] = vald_rutt[i];
    
    int utgangsnod = (ingangsnod == vag_u) ? vag_v : vag_u;
    int slut_framat[NODES], slut_bakom[NODES];
    optimeringsfunktion(utgangsnod, END, slut_framat); optimeringsfunktion(ingangsnod, END, slut_bakom);
    
    int langd_framat = rakna_langd(slut_framat); int langd_bakom = rakna_langd(slut_bakom);
    char riktning_pa_vag = nodriktningsmatris[ingangsnod][utgangsnod];
    
    if (langd_framat < langd_bakom) { for (int i = 0; i < NODES; i++) slutrutt[i] = slut_framat[i]; } 
    else if (langd_bakom < langd_framat) { for (int i = 0; i < NODES; i++) slutrutt[i] = slut_bakom[i]; } 
    else {
        int svangar_framat = rakna_svangar(slut_framat, riktning_pa_vag);
        char motsatt_riktning = nodriktningsmatris[utgangsnod][ingangsnod];
        int svangar_bakom = rakna_svangar(slut_bakom, motsatt_riktning) + 1; 
        int *vald_slut = (svangar_framat <= svangar_bakom) ? slut_framat : slut_bakom;
        for (int i = 0; i < NODES; i++) slutrutt[i] = vald_slut[i];
    }
}

void beslutsfunktion(int rutt_array[], char start_riktning, char beslut_out[]) {
    char nuvarande_riktning = start_riktning;
    int i;
    for (i = 0; rutt_array[i + 1] != STOP; i++){
        int u = rutt_array[i]; int v = rutt_array[i + 1];
        char mal_riktning = nodriktningsmatris[u][v];
        if (nuvarande_riktning == mal_riktning) beslut_out[i] = 'f';
        else if ((nuvarande_riktning == 'n' && mal_riktning == 'e') || (nuvarande_riktning == 'e' && mal_riktning == 's') ||
                 (nuvarande_riktning == 's' && mal_riktning == 'w') || (nuvarande_riktning == 'w' && mal_riktning == 'n')) beslut_out[i] = 'r';
        else if ((nuvarande_riktning == 'n' && mal_riktning == 'w') || (nuvarande_riktning == 'w' && mal_riktning == 's') ||
                 (nuvarande_riktning == 's' && mal_riktning == 'e') || (nuvarande_riktning == 'e' && mal_riktning == 'n')) beslut_out[i] = 'l';
        else beslut_out[i] = 'b';
        nuvarande_riktning = mal_riktning;
    }
    beslut_out[i] = 'X'; 
}

char get_turn_command(char current_dir, char target_dir) {
    if (current_dir == target_dir) return 'f';
    if ((current_dir == 'n' && target_dir == 'e') || (current_dir == 'e' && target_dir == 's') ||
        (current_dir == 's' && target_dir == 'w') || (current_dir == 'w' && target_dir == 'n')) return 'r';
    if ((current_dir == 'n' && target_dir == 'w') || (current_dir == 'w' && target_dir == 's') ||
        (current_dir == 's' && target_dir == 'e') || (current_dir == 'e' && target_dir == 'n')) return 'l';
    return 'b';
}

// =================================================================
// 4. HUVUDPROGRAM
// =================================================================
int main() {
    enable_raw_mode(); 
    init_csv_log(); // Skapa en fräsch loggfil
    
    printf("--- SYSTEMSTART RASPBERRY PI ---\n\r");
    printf(" [A] = Auto (Dijkstra) | [M] = Manuell | [Q] = Avsluta\n\r");
    printf(" I Manuell: [H] = Styr Hjul | [K] = Styr Klo | [S] = Stopp\n\n\r");
    
    init_karta();
    int aktivvag_u = 12; 
    int aktivvag_v = 13; 
    char start_riktning = 's'; 

    berakna_rutter(aktivvag_u, aktivvag_v, start_riktning);
    beslutsfunktion(malrutt, start_riktning, malbeslut);
    
    int ingangs_nod_id = malrutt[rakna_langd(malrutt) - 1]; 
    int utgangs_nod_id = (ingangs_nod_id == aktivvag_u) ? aktivvag_v : aktivvag_u;
    
    char riktning_efter_upphamtning = nodriktningsmatris[ingangs_nod_id][utgangs_nod_id];
    beslutsfunktion(slutrutt, riktning_efter_upphamtning, slutbeslut);

    int file = open("/dev/i2c-1", O_RDWR);
    if (file < 0) {
        printf("\n\r[FEL] Kunde inte öppna /dev/i2c-1! Har du kört med 'sudo'?\n\r");
        return 1; 
    }

    uint8_t buffer_in[8], buffer_out[8];

    // --- TILLSTÅNDSVARIABLER (AUTO) ---
    char* aktuell_rutt = malbeslut; 
    int* aktuella_noder = malrutt; 
    int  beslut_index = 0;
    
    bool var_i_korsning_forra_loopen = false;
    bool roterar_just_nu = false;
    int rotation_timer = 0;    
    bool uppdrag_klart = false;

    char nuvarande_riktning = start_riktning; 
    bool is_heading_to_mid_pickup = false;

    // --- TILLSTÅNDSVARIABLER (MANUELL) ---
    bool is_manual_mode = false;
    char manual_target = 'h'; 
    char manual_action = 's'; 

    // Protokoll-variablerna
    uint8_t out_state = 1; 
    char out_cmd = 'h';    
    char out_action = 's'; 
    uint8_t out_linjefel_offset = 128; 
    int16_t ut_gyro_data = 0;

    printf("Karta och I2C OK! Loggar data till 'robot_data.csv'...\n\r");

    while (!uppdrag_klart) {
        
        // --- A. LÄS TANGENTBORDET (Robust ANSI Escape Sequence-hantering) ---
        char c;
        while (read(STDIN_FILENO, &c, 1) == 1) {
            
            if (c == '\x1b') { 
                char seq[2];
                usleep(20000); 
                
                if (read(STDIN_FILENO, &seq[0], 1) == 1 && seq[0] == '[') {
                    if (read(STDIN_FILENO, &seq[1], 1) == 1) {
                        char arrow = seq[1];
                        if (manual_target == 'h') {
                            if (arrow == 'A') { manual_action = 'f'; printf("\r[Manuell] Kör Framåt\n\r"); }
                            else if (arrow == 'B') { manual_action = 'b'; printf("\r[Manuell] Backar\n\r"); }
                            else if (arrow == 'C') { manual_action = 'r'; printf("\r[Manuell] Svänger Höger\n\r"); }
                            else if (arrow == 'D') { manual_action = 'l'; printf("\r[Manuell] Svänger Vänster\n\r"); }
                        } else if (manual_target == 'a') {
                            if (arrow == 'A') { manual_action = 'p'; printf("\r[Manuell] Klo Plockar\n\r"); }
                            else if (arrow == 'B') { manual_action = 'd'; printf("\r[Manuell] Klo Lämnar\n\r"); }
                        }
                    }
                }
            } 
            else {
                if (c == 'q' || c == 'Q') { uppdrag_klart = true; break; }
                else if (c == 'a' || c == 'A') { is_manual_mode = false; printf("\n\r>>> LÄGE: AUTO <<<\n\r"); }
                else if (c == 'm' || c == 'M') { is_manual_mode = true;  printf("\n\r>>> LÄGE: MANUELL <<<\n\r"); manual_action = 's'; }
                else if (c == 'h' || c == 'H') { manual_target = 'h'; printf("\r[Manuell] Mål: HJUL\n\r"); }
                else if (c == 'k' || c == 'K') { manual_target = 'a'; printf("\r[Manuell] Mål: KLO\n\r"); }
                else if (c == 's' || c == 'S') { manual_action = 's'; printf("\r[Manuell] STOPP\n\r");}
            }
        }

        // --- B. LÄS SENSOR ---
        if (ioctl(file, I2C_SLAVE, 0x11) >= 0) {
            if (read(file, buffer_in, 8) == 8) {
                
                uint8_t stat  = buffer_in[0];
                int16_t dev0  = (int16_t)(buffer_in[1] | (buffer_in[2] << 8));
                int16_t dev1  = (int16_t)(buffer_in[3] | (buffer_in[4] << 8));
                int16_t omega = (int16_t)(buffer_in[6] | (buffer_in[7] << 8));

                int16_t diff = dev0 - dev1; // Riktigt vinkelfel för loggen
                int16_t begransat_diff = diff;
                if (begransat_diff > 127) begransat_diff = 127;
                if (begransat_diff < -128) begransat_diff = -128;
                
                out_linjefel_offset = (uint8_t)(begransat_diff + 128); 
                ut_gyro_data = omega;

                bool korsning_detekterad = (stat & (1 << 3)) != 0; 
                bool linje_fram_hittad = (stat & (1 << 0)) != 0; 

                // --- C. BESLUTSLOGIK ---
                if (is_manual_mode) {
                    out_state = 2; 
                    out_cmd = manual_target;
                    out_action = manual_action;
                    var_i_korsning_forra_loopen = korsning_detekterad; 
                } 
                else {
                    out_state = 1; 
                    
                    if (korsning_detekterad && !var_i_korsning_forra_loopen && !roterar_just_nu) {
                        int nuvarande_nod = aktuella_noder[beslut_index];

                        if (is_heading_to_mid_pickup) {
                            out_cmd = 'a'; out_action = 'p';
                            printf("\r*** MITTEN-KORSNING NÅDD: PLOCKAR VARA (p) ***\n\r");
                            
                            aktuell_rutt = slutbeslut;
                            aktuella_noder = slutrutt;
                            beslut_index = 0;
                            is_heading_to_mid_pickup = false;
                        } 
                        else {
                            char beslut = aktuell_rutt[beslut_index]; 
                            
                            if (beslut == 'X') {
                                if (aktuell_rutt == malbeslut) {
                                    char dir_to_exit = nodriktningsmatris[ingangs_nod_id][utgangs_nod_id];
                                    out_cmd = 'h';
                                    out_action = get_turn_command(nuvarande_riktning, dir_to_exit);
                                    
                                    printf("\r*** INGÅNGSNOD %d NÅDD. SVÄNGER ('%c') MOT MITTEN... ***\n\r", ingangs_nod_id, out_action);
                                    
                                    nuvarande_riktning = dir_to_exit; 
                                    is_heading_to_mid_pickup = true;
                                } else {
                                    out_cmd = 'a'; out_action = 'd';
                                    printf("\r*** END NÅDD (NOD %d)! LÄMNAR VARA (d)... ***\n\r", END);
                                    uppdrag_klart = true;
                                }
                            } else {
                                out_cmd = 'h'; out_action = beslut;
                                printf("\r[KORSNING: NOD %d] Auto skickar: '%c'\n\r", nuvarande_nod, beslut);
                                nuvarande_riktning = nodriktningsmatris[ aktuella_noder[beslut_index] ][ aktuella_noder[beslut_index+1] ];
                                beslut_index++;
                            }
                        }

                        if (out_action == 'l' || out_action == 'r' || out_action == 'b') {
                            roterar_just_nu = true; rotation_timer = 30;
                        }
                    }
                    else if (roterar_just_nu) {
                        if (rotation_timer > 0) rotation_timer--;
                        else if (linje_fram_hittad && !korsning_detekterad) {
                            roterar_just_nu = false; 
                            printf("\r[ROTATION KLAR] Återgår till linjeföljning.\n\r");
                        }
                    }
                    else {
                        out_cmd = 'h'; out_action = 'f';
                    }
                    var_i_korsning_forra_loopen = korsning_detekterad;
                }

                // --- SKRIV TILL LOGGFIL ---
                // Vi skickar in det sanna linjefelet (diff), inte den avklippta/offset-versionen,
                // för att få så bra grafer som möjligt!
                log_data_csv(out_state, diff, omega, out_action);

                // --- D. BYGG OCH SKICKA I2C-PAKET ---
                buffer_out[0] = 0x05;                 
                buffer_out[1] = out_state;            
                buffer_out[2] = (uint8_t)out_cmd;     
                buffer_out[3] = (uint8_t)out_action;  
                buffer_out[4] = out_linjefel_offset;  
                buffer_out[5] = (uint8_t)(ut_gyro_data & 0xFF);   
                buffer_out[6] = (uint8_t)((ut_gyro_data >> 8) & 0xFF); 
                buffer_out[7] = 0xFF;                 

                if (ioctl(file, I2C_SLAVE, 0x12) >= 0) {
                    if (write(file, buffer_out, 8) == 8) {
                        uint8_t buffer_echo[8] = {0};
                        usleep(5000); 
                        if (read(file, buffer_echo, 8) == 8) {
                            log_verifikation(buffer_out, buffer_echo);
                        }
                    }
                }
            }
        }
        usleep(50000); 
    }

    close(file);
    printf("\n\rProgram avslutad snyggt. Glöm inte kolla robot_data.csv!\n\r");
    return 0; 
}