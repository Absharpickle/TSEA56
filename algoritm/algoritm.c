#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#define NODES 26 // 25 noder i rutnätet + 1 startnod
#define START 25 
#define NONE -1
#define STOP -1

char nodriktningsmatris[NODES][NODES];
int  vag[NODES][NODES];
int  rutt_till_vara[NODES];
int  rutt_hem[NODES];
char beslut_till_vara[NODES];
char beslut_hem[NODES];
int  vara_u, vara_v; 

// =================================================================
// 1. KARTA OCH HJÄLPFUNKTIONER
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
    // Koppla Startplatsen till nod 0
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

void hinder_laggtill(int hinder_u, int hinder_v){
    vag[hinder_u][hinder_v] = 0;
    vag[hinder_v][hinder_u] = 0;
}
void hinder_tabort(int hinder_u, int hinder_v){
    vag[hinder_u][hinder_v] = 1;
    vag[hinder_v][hinder_u] = 1;
}
char get_motsatt_dir(char nu) {
    if (nu == 's') return 'n';
    if (nu == 'n') return 's';
    if (nu == 'e') return 'w';
    if (nu == 'w') return 'e';
    return nu; // Om inget matchar
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

// =================================================================
// 2. DIJKSTRA (SÖK ALGORITM)
// =================================================================
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
                int straff = 0;
                if (riktning_in[u] != nasta_dir) straff = 1;
                
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

// =================================================================
// 3. RUTTHANTERING
// =================================================================
void planera_hela_resan(int nuvarande_nod, char nuvarande_dir) {
    int rutt_alt1[NODES];
    int rutt_alt2[NODES];
    
    // 1. Hitta bästa väg TILL varan
    int kostnad1 = hitta_rutt(nuvarande_nod, vara_u, rutt_alt1, nuvarande_dir);
    int kostnad2 = hitta_rutt(nuvarande_nod, vara_v, rutt_alt2, nuvarande_dir);
    
    int ingang, utgang;
    if (kostnad1 <= kostnad2) {
        ingang = vara_u;
        utgang = vara_v;
        memcpy(rutt_till_vara, rutt_alt1, sizeof(rutt_alt1));
    } else {
        ingang = vara_v;
        utgang = vara_u;
        memcpy(rutt_till_vara, rutt_alt2, sizeof(rutt_alt2));
    }

    // Lägg till utgångsnoden i rutt_till_vara för att roboten ska köra in i krysset
    int i = 0;
    while (rutt_till_vara[i] != STOP) i++;
    rutt_till_vara[i] = utgang;
    rutt_till_vara[i+1] = STOP;

    // Bygg besluten för resan till varan
    bygg_beslut(rutt_till_vara, nuvarande_dir, beslut_till_vara);
    
// ... (efter att rutt_till_vara är klar)

    // 2. Planera resan HEM
    char dir_vid_vara = nodriktningsmatris[ingang][utgang]; // Riktningen roboten har i mitten
    
    int cost_utgang = hitta_rutt(utgang, START, rutt_alt1, dir_vid_vara);
    // Vi lägger på ett straff (t.ex. 101) för att markera att vända/backa är dyrare än att bara köra på
    int cost_ingang = hitta_rutt(ingang, START, rutt_alt2, dir_vid_vara) + 100; 

 if (cost_utgang <= cost_ingang) {
        // --- ALTERNATIV: FRAMÅT ---
        memcpy(rutt_hem, rutt_alt1, sizeof(rutt_alt1));
        bygg_beslut(rutt_hem, dir_vid_vara, beslut_hem);
        // Här behövs ingen memmove! bygg_beslut har redan lagt 'f' 
        // om riktningen stämmer, eller 'r'/'l' om utgångsnoden kräver en sväng.
    } 
    else {
        // --- ALTERNATIV: VÄNDA ---
        memcpy(rutt_hem, rutt_alt2, sizeof(rutt_alt2));
        
        // Vi räknar ut riktningen roboten har EFTER att den vänt 180 grader
        char dir_efter_vanding = get_motsatt_dir(dir_vid_vara);
        
        // Bygg besluten utifrån den NYA riktningen
        bygg_beslut(rutt_hem, dir_efter_vanding, beslut_hem);
        
        // Skjut in vändningen 'b' ALLRA först
        int len = strlen(beslut_hem) + 1;
        memmove(&beslut_hem[1], &beslut_hem[0], len);
        beslut_hem[0] = 'b'; 
    }
}
// =================================================================
// 4. UTSKRIFT OCH SIMULERING
// =================================================================
void print_rutt(char label[], int rutt[], char beslut[]) {
    printf("%s | Noder: ", label);
    for (int i = 0; rutt[i] != STOP; i++) printf("%d ", rutt[i]);
    printf("\n%s | Beslut: ", label);
    for (int i = 0; beslut[i] != '\0'; i++) printf("%c ", beslut[i]);
    printf("\n\n");
}

void simulator() {
    vara_u = 6; 
    vara_v = 7; 
    char start_dir = 's'; 
    
    printf("Planerar resa...\n");
    planera_hela_resan(START, start_dir);
    print_rutt("STARTRUTT (Till Varan)", rutt_till_vara, beslut_till_vara);
    print_rutt("SLUTRUTT  (Hemresa)   ", rutt_hem, beslut_hem);
    sleep(1); // En kort paus räcker oftast

    // RESAN TILL VARAN
    printf("Resa till vara: ");
    for(int i = 0; i < NODES; i++) {
        if (beslut_till_vara[i] == 'X') {
            break;
        }
        printf("%c ", beslut_till_vara[i]); // %c krävs för char
        fflush(stdout); // Tvingar terminalen att skriva ut direkt trots sleep
        sleep(5); 
    }
    
    printf("\n[STOPP] Plockar upp vara... (Väntar 5 sekunder)\n");
    sleep(5);

    // RESAN HEM
    printf("Resa hem: ");
    for(int i = 0; i < NODES; i++) {
        if (beslut_hem[i] == 'X') {
            break;
        }
        printf("%c ", beslut_hem[i]); // Använd beslut_hem här
        fflush(stdout);
        sleep(5);
    }
    printf("\n[FRAMME] Hemma vid START!\n");
}


int main() {
    init_karta();
    simulator();
    return 0;
}