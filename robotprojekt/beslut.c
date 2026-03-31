#include <stdio.h>
#include <stdbool.h>

#define NODES 27
#define START 25
#define END 26 
#define INF -1
#define WHITE 0
#define GRAY 1
#define BLACK 2
#define NONE -1
#define STOP -1

// Globala variabler
char nodriktningsmatris[NODES][NODES];
int vag[NODES][NODES];

int malrutt[NODES];
int slutrutt[NODES];

char malbeslut[NODES];
char slutbeslut[NODES];

// --- STRUKTURER FÖR HÅRDVARUKOMMUNIKATION ---
typedef struct {
    bool hinder_detekterat;
    bool vid_korsning;
    int linje_avvikelse;
} Sensordata;

// Dummy-funktion för att skicka I2C
void skicka_i2c_kommando(char kommando) {
    // printf("[I2C] Skickar kommando: %c\n", kommando); // Avkommentera för I2C-debug
}


// --- 1. INITIERING ---
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

        if (kol < 4) {
            vag[i][i + 1] = 1;
            nodriktningsmatris[i][i + 1] = 'e';
        }
        if (kol > 0) {
            vag[i][i - 1] = 1;
            nodriktningsmatris[i][i - 1] = 'w';
        }
        if (rad < 4) {
            vag[i][i + 5] = 1;
            nodriktningsmatris[i][i + 5] = 's';
        }
        if (rad > 0) {
            vag[i][i - 5] = 1;
            nodriktningsmatris[i][i - 5] = 'n';
        }
    }
    
    // START ligger nedanför nod 0 (roboten kör norrut in i gridet)
    vag[START][0] = 1;
    vag[0][START] = 1;
    nodriktningsmatris[START][0] = 's'; 
    nodriktningsmatris[0][START] = 'n'; 
    
    // END ligger nedanför nod 24 (roboten kör söderut ut från gridet)
    vag[24][END] = 1;
    vag[END][24] = 1;
    nodriktningsmatris[24][END] = 's'; 
    nodriktningsmatris[END][24] = 'n'; 
}


// --- 2. RUTTBERÄKNING (Kortaste väg & Beslut) ---
void optimeringsfunktion(int startnod, int slutnod, int rutt_array[]){
    int color[NODES];
    int distance[NODES];
    int prev[NODES];
    int queue[NODES];
    int first = 0, last = 0;

    for (int u = 0; u < NODES; u++){
        color[u] = WHITE;
        distance[u] = INF;
        prev[u] = NONE;
        rutt_array[u] = STOP;
    }

    color[startnod] = GRAY;
    distance[startnod] = 0;
    queue[last++] = startnod;

    while (first < last) {
        int u = queue[first++];
        if (u == slutnod) break;

        for (int v = 0; v < NODES; v++){
            if (vag[u][v] == 1 && color[v] == WHITE){
                color[v] = GRAY;
                distance[v] = distance[u] + 1;
                prev[v] = u;
                queue[last++] = v;
            }
        }
        color[u] = BLACK;
    }

    int temp_rutt[NODES];
    int current = slutnod;
    int count = 0;

    while (current != NONE){
        temp_rutt[count++] = current;
        current = prev[current];
    }
    for (int i = 0; i < count; i++){
        rutt_array[i] = temp_rutt[count - 1 - i];
    }
}

int rakna_langd(int rutt_array[]) {
    int langd = 0;
    while (rutt_array[langd] != STOP) langd++;
    return langd;
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
    
    // Målrutt
    optimeringsfunktion(START, vag_u, rutt_u);
    optimeringsfunktion(START, vag_v, rutt_v);
    
    int langd_u = rakna_langd(rutt_u);
    int langd_v = rakna_langd(rutt_v);
    int ingangsnod;
    
    if (langd_u < langd_v) {
        ingangsnod = vag_u;
    } else if (langd_v < langd_u) {
        ingangsnod = vag_v;
    } else {
        ingangsnod = (rakna_svangar(rutt_u, start_riktning) <= rakna_svangar(rutt_v, start_riktning)) ? vag_u : vag_v;
    }
    
    int *vald_rutt = (ingangsnod == vag_u) ? rutt_u : rutt_v;
    for (int i = 0; i < NODES; i++) malrutt[i] = vald_rutt[i];
    
    // Slutrutt
    int utgangsnod = (ingangsnod == vag_u) ? vag_v : vag_u;
    int slut_framat[NODES], slut_bakom[NODES];
    
    optimeringsfunktion(utgangsnod, END, slut_framat);
    optimeringsfunktion(ingangsnod, END, slut_bakom);
    
    int langd_framat = rakna_langd(slut_framat);
    int langd_bakom = rakna_langd(slut_bakom);
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
        int u = rutt_array[i];
        int v = rutt_array[i + 1];
        char mal_riktning = nodriktningsmatris[u][v];
        
        if (nuvarande_riktning == mal_riktning) {
            beslut_out[i] = 'f';
        } else if ((nuvarande_riktning == 'n' && mal_riktning == 'e') ||
                   (nuvarande_riktning == 'e' && mal_riktning == 's') ||
                   (nuvarande_riktning == 's' && mal_riktning == 'w') ||
                   (nuvarande_riktning == 'w' && mal_riktning == 'n')) {
            beslut_out[i] = 'r';
        } else if ((nuvarande_riktning == 'n' && mal_riktning == 'w') ||
                   (nuvarande_riktning == 'w' && mal_riktning == 's') ||
                   (nuvarande_riktning == 's' && mal_riktning == 'e') ||
                   (nuvarande_riktning == 'e' && mal_riktning == 'n')) {
            beslut_out[i] = 'l';
        } else {
            beslut_out[i] = 'b';
        }
        nuvarande_riktning = mal_riktning;
    }
    beslut_out[i] = 'X'; // Indikerar slutet på beslutskedjan
}


// --- 3. UTFÖRANDEFUNKTION ---
bool utforandefunktion(char beslut_array[], Sensordata sensor) {
    static int n_index = 0; 

    // Stanna vid hinder
    if (sensor.hinder_detekterat) {
        skicka_i2c_kommando('s');
        return false;
    }

    // Är vi framme vid slutet av rutten?
    if (beslut_array[n_index] == 'X') {
        skicka_i2c_kommando('s');
        n_index = 0; // Återställ index inför NÄSTA rutt
        return true; 
    }

    // Agera vid korsning
    if (sensor.vid_korsning) {
        skicka_i2c_kommando(beslut_array[n_index]);
        n_index++; 
    } 
    // Följ linjen om vi inte är i en korsning
    else {
        skicka_i2c_kommando('f'); 
    }
    
    return false; // Har inte nått slutet av rutten än
}


// --- 4. HUVUDPROGRAM (Main Loop) ---
int main() {
    init_karta();

    // 1. Datorn säger: "Hämta varan på vägparet mellan nod 12 och 13"
    int aktivvag_u = 12;
    int aktivvag_v = 13;
    
    // Roboten startar vänd norrut ('n') från START(25) till nod 0.
    char start_riktning = 'n'; 

    printf("--- SYSTEMSTART ---\n");
    
    // 2. Beräkna rutterna
    berakna_rutter(aktivvag_u, aktivvag_v, start_riktning);

    // 3. Omvandla rutterna till körbeslut
    beslutsfunktion(malrutt, start_riktning, malbeslut);
    
    // --- KORREGERAD LOGIK FÖR SLUTRUTTENS STARTRIKTNING ---
    // Vilken nod av 12 och 13 kom vi fram till först?
    int ingangsnod = malrutt[rakna_langd(malrutt) - 1]; 
    int den_andra = (ingangsnod == aktivvag_u) ? aktivvag_v : aktivvag_u;
    
    char riktning_efter_upphamtning;
    if (slutrutt[0] == den_andra) {
        // Om slutrutten börjar på den ANDRA noden, betyder det att vi körde framåt 
        // på vägparet (från ingangsnod till den_andra) och står vända åt det hållet.
        riktning_efter_upphamtning = nodriktningsmatris[ingangsnod][den_andra];
    } else {
        // Om slutrutten börjar på INGÅNGSNODEN, betyder det att vi hämtade varan 
        // och sedan VÄNDE OM. Då står vi vända åt motsatt håll.
        riktning_efter_upphamtning = nodriktningsmatris[den_andra][ingangsnod];
    }
    
    beslutsfunktion(slutrutt, riktning_efter_upphamtning, slutbeslut);


    // ========================================================
    // Utskrift av beräknade rutter och beslut
    // ========================================================
    printf("\n[PLANERING KLAR]\n");
    
    // Skriv ut Målrutt (till vägparet)
    printf("Målrutt (Noder):   ");
    for (int i = 0; malrutt[i] != STOP; i++) {
        printf("%d ", malrutt[i]);
    }
    printf("\nMålrutt (Beslut):  ");
    for (int i = 0; malbeslut[i] != 'X'; i++) {
        printf("%c ", malbeslut[i]);
    }
    
    // Skriv ut Slutrutt (från vägparet till END)
    printf("\n\nSlutrutt (Noder):  ");
    for (int i = 0; slutrutt[i] != STOP; i++) {
        printf("%d ", slutrutt[i]);
    }
    printf("\nSlutrutt (Beslut): ");
    for (int i = 0; slutbeslut[i] != 'X'; i++) {
        printf("%c ", slutbeslut[i]);
    }
    printf("\n========================================================\n\n");


    printf("Redo att köra mot vägparet (%d, %d). Startar Autonom loop.\n", aktivvag_u, aktivvag_v);

    // --- SYSTEMLOOP PÅ RASPBERRY PI ---
    int test_korsningar_passerade = 0; 

    while (true) {
        Sensordata sensor;
        sensor.hinder_detekterat = false;
        sensor.vid_korsning = false;

        test_korsningar_passerade++;
        if (test_korsningar_passerade % 3 == 0) {
            sensor.vid_korsning = true;
            printf("\n-- Sensor: Korsning upptäckt! --");
        }

        bool framme_vid_mal = utforandefunktion(malbeslut, sensor);
        
        if (framme_vid_mal) {
            printf("\n\n*** FRAMME VID VÄGPARET! ***\n");
            printf("Initierar upphämtning av vara...\n");
            break; 
        }
    }

    printf("\nVara upphämtad. Startar loop för hemfärd till END...\n");
    test_korsningar_passerade = 0;

    while (true) {
        Sensordata sensor;
        sensor.hinder_detekterat = false;
        sensor.vid_korsning = false;

        test_korsningar_passerade++;
        if (test_korsningar_passerade % 3 == 0) {
            sensor.vid_korsning = true;
            printf("\n-- Sensor: Korsning upptäckt! --");
        }

        bool framme_vid_end = utforandefunktion(slutbeslut, sensor);
        
        if (framme_vid_end) {
            printf("\n\n*** FRAMME VID END! UPPDRAG SLUTFÖRT! ***\n");
            break;
        }
    }

    return 0;
}