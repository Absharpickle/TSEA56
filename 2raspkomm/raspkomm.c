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

// =================================================================
// 1. TERMINAL-HANTERING
// =================================================================
struct termios orig_termios;
void disable_raw_mode() { tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios); }
void enable_raw_mode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(disable_raw_mode);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

// =================================================================
// 2. DIJKSTRA & RUTTPLANERING
// =================================================================
#define NODES 27
#define START 25
#define END 26 
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

// (Dijkstra-funktioner optimeringsfunktion, rakna_langd, etc. antas vara samma som tidigare)
// ... [Klistra in din befintliga optimeringsfunktion, berakna_rutter och beslutsfunktion här] ...

// Hjälpfunktion för att få fram sväng-kommando baserat på nuvarande och målsatt riktning
char get_turn_command(char current_dir, char target_dir) {
    if (current_dir == target_dir) return 'f';
    if ((current_dir == 'n' && target_dir == 'e') || (current_dir == 'e' && target_dir == 's') ||
        (current_dir == 's' && target_dir == 'w') || (current_dir == 'w' && target_dir == 'n')) return 'r';
    if ((current_dir == 'n' && target_dir == 'w') || (current_dir == 'w' && target_dir == 's') ||
        (current_dir == 's' && target_dir == 'e') || (current_dir == 'e' && target_dir == 'n')) return 'l';
    return 'b';
}

// =================================================================
// 3. HUVUDPROGRAM
// =================================================================
int main() {
    enable_raw_mode();
    init_karta();

    int aktivvag_u = 12, aktivvag_v = 13;
    char start_riktning = 's';

    // Beräkna rutter
    // berakna_rutter(aktivvag_u, aktivvag_v, start_riktning); ...
    // [Antag att malbeslut och slutbeslut är genererade]

    int file = open("/dev/i2c-1", O_RDWR);
    if (file < 0) return 1;

    uint8_t buffer_in[8], buffer_out[8];
    int beslut_index = 0;
    bool var_i_korsning_forra_loopen = false, roterar_just_nu = false;
    int rotation_timer = 0;
    bool uppdrag_klart = false;

    // --- NYA TILLSTÅND FÖR MITT-KORSNING ---
    bool is_heading_to_mid_pickup = false;
    int ingangs_nod_id = 12; // Exempel
    int utgangs_nod_id = 13; // Exempel

    bool is_manual_mode = false;
    char manual_target = 'h', manual_action = 's';
    uint8_t out_state = 1;
    char out_cmd = 'h', out_action = 's';

    printf("--- MASTER STARTAD ---\n\r");

    while (!uppdrag_klart) {
        // 1. LÄS TANGENTBORD
        char c;
        if (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == 'q') break;
            if (c == 'm') { is_manual_mode = true; printf("\rLÄGE: MANUELL\n\r"); }
            if (c == 'a') { is_manual_mode = false; printf("\rLÄGE: AUTO\n\r"); }
            // ... (hantera piltangenter för manuell styrning här)
        }

        // 2. LÄS SENSOR
        ioctl(file, I2C_SLAVE, 0x11);
        if (read(file, buffer_in, 8) == 8) {
            uint8_t stat = buffer_in[0];
            int16_t dev0 = (int16_t)(buffer_in[1] | (buffer_in[2] << 8));
            int16_t dev1 = (int16_t)(buffer_in[3] | (buffer_in[4] << 8));
            int16_t omega = (int16_t)(buffer_in[6] | (buffer_in[7] << 8));

            bool korsning_detekterad = (stat & (1 << 3)) != 0;
            bool linje_fram_hittad = (stat & (1 << 0)) != 0;

            if (is_manual_mode) {
                out_state = 2; out_cmd = manual_target; out_action = manual_action;
            } else {
                out_state = 1;
                
                // --- AUTO LOGIK MED MITT-KORSNING ---
                if (korsning_detekterad && !var_i_korsning_forra_loopen && !roterar_just_nu) {
                    
                    if (is_heading_to_mid_pickup) {
                        // VI ÄR NU I MITTEN-KORSNINGEN!
                        out_cmd = 'a'; out_action = 'p';
                        printf("\r*** MITTEN-KORSNING UPPTÄCKT: PLOCKAR VARA (p) ***\n\r");
                        
                        // Växla till hemfärds-rutten
                        // aktuell_rutt = slutbeslut;
                        beslut_index = 0;
                        is_heading_to_mid_pickup = false;
                    } 
                    else {
                        char beslut = malbeslut[beslut_index]; // Förenklat för exempel
                        
                        if (beslut == 'X') {
                            // Vi har nått ingångsnoden! Nu ska vi köra mot utgångsnoden.
                            char dir_to_exit = nodriktningsmatris[ingangs_nod_id][utgangs_nod_id];
                            // Hitta nuvarande riktning (bör vara samma som mal_riktning från sista steget)
                            char cur_dir = 'n'; // Denna bör spåras dynamiskt i din beslutsfunktion
                            
                            out_cmd = 'h';
                            out_action = get_turn_command(cur_dir, dir_to_exit);
                            is_heading_to_mid_pickup = true;
                            
                            printf("\r*** INGÅNGSNOD NÅDD. KÖR MOT MITTEN... ***\n\r");
                        } else {
                            // Vanlig ruttkörning
                            out_cmd = 'h'; out_action = beslut;
                            beslut_index++;
                        }
                    }

                    if (out_action == 'l' || out_action == 'r' || out_action == 'b') {
                        roterar_just_nu = true; rotation_timer = 30;
                    }
                }
                else if (roterar_just_nu) {
                    if (rotation_timer > 0) rotation_timer--;
                    else if (linje_fram_hittad && !korsning_detekterad) roterar_just_nu = false;
                } else {
                    out_cmd = 'h'; out_action = 'f';
                }
            }

            // 3. SKICKA TILL STYRMODUL
            buffer_out[0] = 0x05;
            buffer_out[1] = out_state;
            buffer_out[2] = (uint8_t)out_cmd;
            buffer_out[3] = (uint8_t)out_action;
            buffer_out[4] = (uint8_t)((dev0 - dev1) + 128); // Linjefel offset binary
            buffer_out[5] = (uint8_t)(omega & 0xFF);
            buffer_out[6] = (uint8_t)((omega >> 8) & 0xFF);
            buffer_out[7] = 0xFF;

            ioctl(file, I2C_SLAVE, 0x12);
            write(file, buffer_out, 8);
            var_i_korsning_forra_loopen = korsning_detekterad;
        }
        usleep(50000);
    }
    return 0;
}