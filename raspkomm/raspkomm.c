#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <time.h>

// =================================================================
// 1. GRAF- OCH RUTTPLANERING (Dijkstra)
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
int malrutt[NODES];
int slutrutt[NODES];
char malbeslut[NODES];
char slutbeslut[NODES];

void init_karta(){
    for (int i = 0; i < NODES; i++){
        for(int j = 0; j < NODES; j++){
            vag[i][j] = 0;
            nodriktningsmatris[i][j] = ' ';
        }
    }
    for (int i = 0; i < 25; i++){
        int rad = i / 5;
        int kol = i % 5;
        if (kol < 4) { vag[i][i + 1] = 1; nodriktningsmatris[i][i + 1] = 'e'; }
        if (kol > 0) { vag[i][i - 1] = 1; nodriktningsmatris[i][i - 1] = 'w'; }
        if (rad < 4) { vag[i][i + 5] = 1; nodriktningsmatris[i][i + 5] = 's'; }
        if (rad > 0) { vag[i][i - 5] = 1; nodriktningsmatris[i][i - 5] = 'n'; }
    }
    vag[START][0] = 1; vag[0][START] = 1;
    nodriktningsmatris[START][0] = 's'; nodriktningsmatris[0][START] = 'n'; 
    vag[24][END] = 1; vag[END][24] = 1;
    nodriktningsmatris[24][END] = 's'; nodriktningsmatris[END][24] = 'n'; 
}

void optimeringsfunktion(int startnod, int slutnod, int rutt_array[]){
    int color[NODES], distance[NODES], prev[NODES], queue[NODES];
    int first = 0, last = 0;

    for (int u = 0; u < NODES; u++){
        color[u] = WHITE; distance[u] = INF; prev[u] = NONE; rutt_array[u] = STOP;
    }
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
    while (current != NONE){
        temp_rutt[count++] = current; current = prev[current];
    }
    for (int i = 0; i < count; i++){ rutt_array[i] = temp_rutt[count - 1 - i]; }
}

int rakna_langd(int rutt_array[]) {
    int langd = 0; while (rutt_array[langd] != STOP) langd++; return langd;
}

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
    
    if (langd_framat < langd_bakom) {
        for (int i = 0; i < NODES; i++) slutrutt[i] = slut_framat[i];
    } else if (langd_bakom < langd_framat) {
        for (int i = 0; i < NODES; i++) slutrutt[i] = slut_bakom[i];
    } else {
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

// =================================================================
// 2. LOGGFUNKTION FÖR SYSTEMET
// =================================================================
void log_robot_status(uint8_t fall, char cmd, int8_t vinkel, const char* status) {
    FILE *f = fopen("robot_logg.txt", "a");
    if (f == NULL) return;
    time_t now = time(NULL); struct tm *t = localtime(&now);
    
    fprintf(f, "[%02d:%02d:%02d] [%s] Fall: %d | Cmd: %c | Vinkelfel: %d\n", 
            t->tm_hour, t->tm_min, t->tm_sec, status, fall, cmd, vinkel);
    fclose(f);
}

// =================================================================
// 3. HUVUDPROGRAM (I2C + AUTONOM LOOP)
// =================================================================
int main() {
    printf("--- SYSTEMSTART RASPBERRY PI ---\n");
    
    // --- 3.1 INITIERA RUTTER ---
    init_karta();
    int aktivvag_u = 12; // Mål-nod 1 (Byt ut till de noder ni tar emot från GUI)
    int aktivvag_v = 13; // Mål-nod 2
    char start_riktning = 'n'; 

    berakna_rutter(aktivvag_u, aktivvag_v, start_riktning);
    beslutsfunktion(malrutt, start_riktning, malbeslut);
    
    int ingangsnod = malrutt[rakna_langd(malrutt) - 1]; 
    int den_andra = (ingangsnod == aktivvag_u) ? aktivvag_v : aktivvag_u;
    char riktning_efter_upphamtning = (slutrutt[0] == den_andra) ? 
        nodriktningsmatris[ingangsnod][den_andra] : nodriktningsmatris[den_andra][ingangsnod];
    beslutsfunktion(slutrutt, riktning_efter_upphamtning, slutbeslut);

    printf("Målrutt (Beslut):  ");
    for (int i = 0; malbeslut[i] != 'X'; i++) printf("%c ", malbeslut[i]);
    printf("\nSlutrutt (Beslut): ");
    for (int i = 0; slutbeslut[i] != 'X'; i++) printf("%c ", slutbeslut[i]);
    printf("\n\n");

    // --- 3.2 INITIERA I2C ---
    int file;
    if ((file = open("/dev/i2c-1", O_RDWR)) < 0) {
        printf("Fel: Kunde inte öppna I2C-bussen.\n");
        return 1;
    }

    uint8_t buffer_in[8];
    uint8_t buffer_out[8];

    // --- 3.3 TILLSTÅNDSVARIABLER FÖR KÖRNING ---
    char* aktuell_rutt = malbeslut; 
    int beslut_index = 0;
    bool var_i_korsning_forra_loopen = false;
    
    bool roterar_just_nu = false;
    uint8_t aktuellt_fall = 2; // Startar i linjeföljningsläge (Fall 2)
    char skickat_kommando = 'f';
    bool uppdrag_klart = false;

    printf("Startar Autonom I2C-loop mot vägpar (%d, %d)...\n", aktivvag_u, aktivvag_v);

    while (!uppdrag_klart) {
        // --- A: LÄS FRÅN SENSOR (0x11) ---
        ioctl(file, I2C_SLAVE, 0x11);
        if (read(file, buffer_in, 8) == 8) {
            
            // Plocka ut variabler (JUSTERA BYTE-INDEX HÄR OM DET BEHÖVS)
            uint8_t status_flags   = buffer_in[1];
            int8_t avvikelse_fram  = (int8_t)buffer_in[2];
            int8_t avvikelse_bak   = (int8_t)buffer_in[3];
            int8_t rotations_hastighet = (int8_t)buffer_in[4]; // Byte 4 = Gyro?
            uint8_t antal_aktiva_lampor = buffer_in[6];        // Byte 6 = Antal tända lampor?

            int8_t vinkel_fel = avvikelse_fram - avvikelse_bak;
            bool hinder_detekterat = (status_flags & (1 << 2)) != 0; // Kollar bit 2
            bool korsning_detekterad = (antal_aktiva_lampor > 4);    // Antar >4 lampor betyder korsning

            // --- B: UTVÄRDERA TILLSTÅND OCH VÄLJ FALL ---
            
            // Prioritet 1: Hinder!
            if (hinder_detekterat && !roterar_just_nu) {
                aktuellt_fall = 1;
                skickat_kommando = 'b';
                roterar_just_nu = true;
                printf("[HINDER] Skickar rotera-bakåt (b)\n");
                log_robot_status(aktuellt_fall, skickat_kommando, vinkel_fel, "HINDER");
            } 
            
            // Prioritet 2: Ny korsning!
            else if (korsning_detekterad && !var_i_korsning_forra_loopen && !roterar_just_nu) {
                aktuellt_fall = 1;
                skickat_kommando = aktuell_rutt[beslut_index];
                
                // Om vi nått 'X' är vi vid målet för rutten
                if (skickat_kommando == 'X') {
                    if (aktuell_rutt == malbeslut) {
                        skickat_kommando = 'v'; // Plocka upp vara
                        printf("\n*** FRAMME VID VÄGPARET! HÄMTAR VARA (v)... ***\n");
                        aktuell_rutt = slutbeslut;
                        beslut_index = 0;
                        // OBS: Du kanske vill lägga in en sleep() här eller låsa i ett 
                        // "plockar_vara"-state tills styrmodulen svarar att klon är klar.
                    } else {
                        skickat_kommando = 'a'; // Avlämning
                        printf("\n*** FRAMME VID END! LÄMNAR VARA (a)... ***\n");
                        uppdrag_klart = true;
                    }
                } else {
                    printf("[KORSNING %d] Skickar kommando: '%c'\n", beslut_index, skickat_kommando);
                    beslut_index++;
                    // Om det är en sväng, gå in i rotations-läge
                    if (skickat_kommando == 'l' || skickat_kommando == 'r' || skickat_kommando == 'b') {
                        roterar_just_nu = true; 
                    }
                }
                log_robot_status(aktuellt_fall, skickat_kommando, vinkel_fel, "KORSNING");
            }

            // Prioritet 3: Pågående rotation (Fall 3)
            else if (roterar_just_nu) {
                aktuellt_fall = 3;
                
                // Villkor för att svängen är klar (exempel: 1-3 lampor ser linjen och ej i en bred korsning)
                if (antal_aktiva_lampor > 0 && antal_aktiva_lampor < 4 && !korsning_detekterad) {
                    roterar_just_nu = false; // Svängen är färdig, återgå till Fall 2 nästa loop!
                    printf("[ROTATION KLAR] Återgår till linjeföljning.\n");
                }
            }

            // Prioritet 4: Linjeföljning (Fall 2)
            else {
                aktuellt_fall = 2;
                skickat_kommando = '-'; // Används inte i Fall 2
            }

            // Spara korsningsstatusen för att bara läsa av *en* gång per korsning
            var_i_korsning_forra_loopen = korsning_detekterad;


            // --- C: BYGG I2C-PAKETET ---
            buffer_out[0] = 0x05;             // Startbyte
            buffer_out[1] = aktuellt_fall;    // ID (1, 2, 3)
            
            // Nollställ för säkerhets skull
            buffer_out[2] = 0x00; buffer_out[3] = 0x00; buffer_out[4] = 0x00; 
            buffer_out[5] = 0x00; buffer_out[6] = 0x00;

            if (aktuellt_fall == 1) {
                // FALL 1: Nytt kommando
                buffer_out[2] = (uint8_t)skickat_kommando; 
            } 
            else if (aktuellt_fall == 2) {
                // FALL 2: Linjeföljning
                buffer_out[2] = (uint8_t)vinkel_fel;      
                buffer_out[3] = (uint8_t)avvikelse_fram;  
                buffer_out[4] = (uint8_t)avvikelse_bak;   
            } 
            else if (aktuellt_fall == 3) {
                // FALL 3: Rotation
                buffer_out[2] = (uint8_t)rotations_hastighet; 
            }

            buffer_out[7] = 0xFF; // Stoppbyte

            // --- D: SKICKA TILL STYRMODUL (0x12) ---
            ioctl(file, I2C_SLAVE, 0x12);
            write(file, buffer_out, 8);
            
        } else {
            printf("Fel: Kunde inte läsa från sensormodulen (0x11).\n");
        }
        
        usleep(50000); // 50ms paus (20 Hz loop)
    }

    close(file);
    printf("Program avslutad snyggt.\n");
    return 0;
}