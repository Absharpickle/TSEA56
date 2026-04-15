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
#include <termios.h>
#include <sys/select.h>

// =================================================================
// 1. TERMINAL-HANTERING (Bulletproof kbhit)
// =================================================================
struct termios orig_termios;

void disable_raw_mode() { 
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios); 
}

void enable_raw_mode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(disable_raw_mode); // Återställ alltid terminalen vid krasch/exit
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON); // Stäng av eko och Enter-krav
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

// Robust funktion för att kolla om en tangent är tryckt (utan att hänga systemet)
int kbhit() {
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

// =================================================================
// 2. DIJKSTRA & RUTTPLANERING (Samma som innan)
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

char get_turn_command(char current_dir, char target_dir) {
    if (current_dir == target_dir) return 'f';
    if ((current_dir == 'n' && target_dir == 'e') || (current_dir == 'e' && target_dir == 's') ||
        (current_dir == 's' && target_dir == 'w') || (current_dir == 'w' && target_dir == 'n')) return 'r';
    if ((current_dir == 'n' && target_dir == 'w') || (current_dir == 'w' && target_dir == 's') ||
        (current_dir == 's' && target_dir == 'e') || (current_dir == 'e' && target_dir == 'n')) return 'l';
    return 'b';
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
// 3. HUVUDPROGRAM
// =================================================================
int main() {
    enable_raw_mode(); 
    
    // 1. Skriv ut DIREKT så vi vet att programmet lever
    printf("--- SYSTEMSTART RASPBERRY PI ---\n\r");
    printf("TANGENTBORDSKONTROLLER:\n\r");
    printf(" [A] = Auto Läge (Dijkstra)\n\r");
    printf(" [M] = Manuell Läge\n\r");
    printf("     -> [H] Styr Hjul (Pilar)\n\r");
    printf("     -> [K] Styr Klo  (Pilar Upp/Ner)\n\r");
    printf("     -> [S] Stoppa rörelse\n\r");
    printf(" [Q] = Avsluta programmet\n\n\r");
    
    init_karta();
    int aktivvag_u = 12; 
    int aktivvag_v = 13; 
    char start_riktning = 's'; 

    berakna_rutter(aktivvag_u, aktivvag_v, start_riktning);
    
    // Tydlig I2C-felhantering
    int file = open("/dev/i2c-1", O_RDWR);
    if (file < 0) {
        printf("\n\r[FEL] Kunde inte öppna /dev/i2c-1! Har du kört med 'sudo'?\n\r");
        return 1; 
    }

    uint8_t buffer_in[8];
    uint8_t buffer_out[8];

    // --- Tillståndsvariabler ---
    int beslut_index = 0;
    bool var_i_korsning_forra_loopen = false;
    bool roterar_just_nu = false;
    int rotation_timer = 0;    
    bool uppdrag_klart = false;

    bool is_heading_to_mid_pickup = false;
    int ingangs_nod_id = malrutt[rakna_langd(malrutt) - 1];
    int utgangs_nod_id = (ingangs_nod_id == aktivvag_u) ? aktivvag_v : aktivvag_u;

    bool is_manual_mode = false;
    char manual_target = 'h'; 
    char manual_action = 's'; 

    uint8_t out_state = 1; 
    char out_cmd = 'h';    
    char out_action = 's'; 
    uint8_t out_linjefel_offset = 128; 
    int16_t ut_gyro_data = 0;

    printf("Karta och I2C OK! Startar loop...\n\r");

    while (!uppdrag_klart) {
        
        // =================================================================
        // A. LÄS TANGENTBORDET (Robust)
        // =================================================================
        while (kbhit()) {
            char c = getchar(); // Läs tryckt knapp
            
            if (c == 'q' || c == 'Q') { uppdrag_klart = true; break; }
            if (c == 'a' || c == 'A') { is_manual_mode = false; printf("\n\r>>> LÄGE: AUTO <<<\n\r"); }
            if (c == 'm' || c == 'M') { is_manual_mode = true;  printf("\n\r>>> LÄGE: MANUELL <<<\n\r"); manual_action = 's'; }
            if (c == 'h' || c == 'H') { manual_target = 'h'; printf("\r[Manuell] Mål satt till: HJUL\n\r"); }
            if (c == 'k' || c == 'K') { manual_target = 'a'; printf("\r[Manuell] Mål satt till: KLO\n\r"); }
            if (c == 's' || c == 'S') { manual_action = 's'; printf("\r[Manuell] STOPP\n\r");}

            // Fånga piltangenter (Esc -> [ -> A/B/C/D)
            if (c == '\x1b') { 
                usleep(5000); // Ge tangentbordet 5ms att skicka resten av piltangenten
                if (kbhit()) {
                    char seq1 = getchar();
                    if (seq1 == '[' && kbhit()) {
                        char arrow = getchar();
                        
                        if (manual_target == 'h') {
                            if (arrow == 'A') { manual_action = 'f'; printf("\r[Manuell] Kör Framåt\n\r"); }
                            if (arrow == 'B') { manual_action = 'b'; printf("\r[Manuell] Backar\n\r"); }
                            if (arrow == 'C') { manual_action = 'r'; printf("\r[Manuell] Svänger Höger\n\r"); }
                            if (arrow == 'D') { manual_action = 'l'; printf("\r[Manuell] Svänger Vänster\n\r"); }
                        } else if (manual_target == 'a') {
                            if (arrow == 'A') { manual_action = 'p'; printf("\r[Manuell] Klo Plockar\n\r"); }
                            if (arrow == 'B') { manual_action = 'd'; printf("\r[Manuell] Klo Lämnar\n\r"); }
                        }
                    }
                }
            }
        }

        // =================================================================
        // B. LÄS SENSOR (0x11)
        // =================================================================
        if (ioctl(file, I2C_SLAVE, 0x11) >= 0) {
            if (read(file, buffer_in, 8) == 8) {
                
                uint8_t stat  = buffer_in[0];
                int16_t dev0  = (int16_t)(buffer_in[1] | (buffer_in[2] << 8));
                int16_t dev1  = (int16_t)(buffer_in[3] | (buffer_in[4] << 8));
                int16_t omega = (int16_t)(buffer_in[6] | (buffer_in[7] << 8));

                int16_t diff = dev0 - dev1;
                if (diff > 127) diff = 127;
                if (diff < -128) diff = -128;
                
                out_linjefel_offset = (uint8_t)(diff + 128); 
                ut_gyro_data = omega;

                bool korsning_detekterad = (stat & (1 << 3)) != 0; 
                bool linje_fram_hittad = (stat & (1 << 0)) != 0; 

                // =================================================================
                // C. BESLUTSLOGIK
                // =================================================================
                if (is_manual_mode) {
                    out_state = 2; // Manuell
                    out_cmd = manual_target;
                    out_action = manual_action;
                    var_i_korsning_forra_loopen = korsning_detekterad; 
                } 
                else {
                    out_state = 1; // Auto
                    
                    if (korsning_detekterad && !var_i_korsning_forra_loopen && !roterar_just_nu) {
                        
                        if (is_heading_to_mid_pickup) {
                            out_cmd = 'a'; out_action = 'p';
                            printf("\r*** MITTEN-KORSNING UPPTÄCKT: PLOCKAR VARA (p) ***\n\r");
                            beslut_index = 0;
                            is_heading_to_mid_pickup = false;
                        } 
                        else {
                            char beslut = malbeslut[beslut_index]; 
                            
                            if (beslut == 'X') {
                                char dir_to_exit = nodriktningsmatris[ingangs_nod_id][utgangs_nod_id];
                                char cur_dir = 'n'; // Anpassa dynamiskt om möjligt
                                
                                out_cmd = 'h';
                                out_action = get_turn_command(cur_dir, dir_to_exit);
                                is_heading_to_mid_pickup = true;
                                printf("\r*** INGÅNGSNOD NÅDD. KÖR MOT MITTEN... ***\n\r");
                            } else {
                                out_cmd = 'h'; out_action = beslut;
                                printf("\r[KORSNING] Auto skickar: '%c'\n\r", beslut);
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

                // =================================================================
                // D. BYGG OCH SKICKA I2C-PAKET
                // =================================================================
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
        usleep(50000); // 50ms vila
    }

    close(file);
    printf("\n\rProgram avslutad snyggt.\n\r");
    return 0; 
}