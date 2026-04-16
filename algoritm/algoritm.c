#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#define NODES 27
#define START 25 
#define END 26   
#define NONE -1
#define STOP -1

char nodriktningsmatris[NODES][NODES];
int vag[NODES][NODES];

int malrutt[NODES], slutrutt[NODES];
char malbeslut[NODES], slutbeslut[NODES];
int mal_u, mal_v; 

// =================================================================
// 1. KARTA & HJÄLPFUNKTIONER
// =================================================================
void init_karta(){
    memset(vag, 0, sizeof(vag));
    memset(nodriktningsmatris, ' ', sizeof(nodriktningsmatris));
    for (int i = 0; i < 25; i++){
        if (i % 5 < 4) { vag[i][i+1] = 1; nodriktningsmatris[i][i+1] = 'e'; } 
        if (i % 5 > 0) { vag[i][i-1] = 1; nodriktningsmatris[i][i-1] = 'w'; } 
        if (i / 5 < 4) { vag[i][i+5] = 1; nodriktningsmatris[i][i+5] = 's'; } 
        if (i / 5 > 0) { vag[i][i-5] = 1; nodriktningsmatris[i][i-5] = 'n'; } 
    }
    vag[START][0] = vag[0][START] = vag[24][END] = vag[END][24] = 1;
    nodriktningsmatris[START][0] = nodriktningsmatris[24][END] = 's';
    nodriktningsmatris[0][START] = nodriktningsmatris[END][24] = 'n';
}

void stang_av_vag(int u, int v) {
    if (u == NONE || v == NONE) return;
    vag[u][v] = 0;
    vag[v][u] = 0;
}

char get_turn(char cur, char next) {
    if (cur == next) return 'f';
    if ((cur=='n'&&next=='e') || (cur=='e'&&next=='s') || (cur=='s'&&next=='w') || (cur=='w'&&next=='n')) return 'r';
    if ((cur=='n'&&next=='w') || (cur=='w'&&next=='s') || (cur=='s'&&next=='e') || (cur=='e'&&next=='n')) return 'l';
    return 'b';
}

char get_motsatt_riktning(char dir) {
    if (dir == 'n') return 's';
    if (dir == 's') return 'n';
    if (dir == 'e') return 'w';
    if (dir == 'w') return 'e';
    return ' ';
}

void bygg_beslut(int rutt[], char start_dir, char beslut[]) {
    char dir = start_dir;
    int i;
    for (i = 0; rutt[i + 1] != STOP; i++) {
        char next_dir = nodriktningsmatris[rutt[i]][rutt[i + 1]];
        beslut[i] = get_turn(dir, next_dir);
        dir = next_dir;
    }
    beslut[i] = 'X'; 
}

// =================================================================
// 2. DIJKSTRA 
// =================================================================
int optimeringsfunktion(int startnod, int slutnod, int rutt[], char start_riktning) {
    int cost[NODES], prev[NODES];
    char dir_arrived[NODES];
    bool visited[NODES] = {false};

    for (int i = 0; i < NODES; i++) { cost[i] = 999999; prev[i] = NONE; rutt[i] = STOP; }
    cost[startnod] = 0; dir_arrived[startnod] = start_riktning;

    for (int count = 0; count < NODES; count++) {
        int min_cost = 999999, u = -1;
        for (int i = 0; i < NODES; i++) {
            if (!visited[i] && cost[i] < min_cost) { min_cost = cost[i]; u = i; }
        }
        if (u == -1 || u == slutnod) break;
        visited[u] = true;

        for (int v = 0; v < NODES; v++) {
            if (vag[u][v] && !visited[v]) {
                char move_dir = nodriktningsmatris[u][v];
                int alt = cost[u] + 100 + (dir_arrived[u] == move_dir ? 0 : 1);
                if (alt < cost[v]) { cost[v] = alt; prev[v] = u; dir_arrived[v] = move_dir; }
            }
        }
    }

    int temp[NODES], c = 0, curr = slutnod;
    while (curr != NONE) { temp[c++] = curr; curr = prev[curr]; }
    for (int i = 0; i < c; i++) rutt[i] = temp[c - 1 - i];

    return cost[slutnod]; 
}

// =================================================================
// 3. DYNAMISK RUTTBERÄKNING
// =================================================================
void uppdatera_fas1_och_2(int startnod, char start_riktning) {
    int rutt_u[NODES], rutt_v[NODES];
    int cost_u = optimeringsfunktion(startnod, mal_u, rutt_u, start_riktning);
    int cost_v = optimeringsfunktion(startnod, mal_v, rutt_v, start_riktning);
    
    int ingang = (cost_u <= cost_v) ? mal_u : mal_v;
    int utgang = (ingang == mal_u) ? mal_v : mal_u;
    int *vald_in = (cost_u <= cost_v) ? rutt_u : rutt_v;

    int i;
    for (i = 0; vald_in[i] != STOP; i++) malrutt[i] = vald_in[i];
    malrutt[i] = utgang; 
    malrutt[i+1] = STOP;
    bygg_beslut(malrutt, start_riktning, malbeslut);

    int slut_framat[NODES], slut_bakom[NODES];
    int cost_fram = optimeringsfunktion(utgang, END, slut_framat, nodriktningsmatris[ingang][utgang]); 
    int cost_bak  = optimeringsfunktion(ingang, END, slut_bakom, nodriktningsmatris[utgang][ingang]) + 1; 
    
    if (cost_fram <= cost_bak) {
        slutrutt[0] = ingang; 
        for (i = 0; slut_framat[i] != STOP; i++) slutrutt[i+1] = slut_framat[i];
        slutrutt[i+1] = STOP;
        bygg_beslut(slutrutt, nodriktningsmatris[ingang][utgang], slutbeslut);
    } else {
        slutrutt[0] = utgang; 
        for (i = 0; slut_bakom[i] != STOP; i++) slutrutt[i+1] = slut_bakom[i];
        slutrutt[i+1] = STOP;
        bygg_beslut(slutrutt, nodriktningsmatris[utgang][ingang], slutbeslut);
    }
}

void uppdatera_fas2(int startnod, char start_riktning) {
    optimeringsfunktion(startnod, END, slutrutt, start_riktning);
    bygg_beslut(slutrutt, start_riktning, slutbeslut);
}

// =================================================================
// 4. DATA UTSKRIFT OCH HINDERSIMULATOR
// =================================================================
void print_fas1() {
    printf("FAS 1 | Noder: ");
    for (int i = 0; malrutt[i] != STOP; i++) {
        if (malrutt[i+1] == STOP) printf("[MITTEN]\n");
        else printf("%d ", malrutt[i]);
    }
    printf("FAS 1 | Kommandon: ");
    for (int i = 0; malbeslut[i] != '\0'; i++) printf("%c ", malbeslut[i]);
    printf("\n\n");
}

void print_fas2(bool offset) {
    printf("FAS 2 | Noder: [MITTEN] ");
    int start_i = offset ? 1 : 0;
    for (int i = start_i; slutrutt[i] != STOP; i++) printf("%d ", slutrutt[i]);
    printf("\nFAS 2 | Kommandon: ");
    for (int i = 0; slutbeslut[i] != '\0'; i++) printf("%c ", slutbeslut[i]);
    printf("\n\n");
}

void kolla_och_applicera_hinder_fas1(int h_u, int h_v) {
    if (h_u == NONE || h_v == NONE) return;
    for (int i = 0; malrutt[i+1] != STOP; i++) {
        if ((malrutt[i] == h_u && malrutt[i+1] == h_v) || (malrutt[i] == h_v && malrutt[i+1] == h_u)) {
            int nuvarande_nod = malrutt[i];
            stang_av_vag(h_u, h_v);
            char riktning = nodriktningsmatris[nuvarande_nod][malrutt[i+1]];
            uppdatera_fas1_och_2(nuvarande_nod, get_motsatt_riktning(riktning));
            printf(">>> HINDER %d-%d HITTTAT! NY RUTT BERÄKNAD:\n", h_u, h_v);
            print_fas1(); print_fas2(true);
            return;
        }
    }
}

void kolla_och_applicera_hinder_fas2(int h_u, int h_v) {
    if (h_u == NONE || h_v == NONE) return;
    for (int i = 0; slutrutt[i+1] != STOP; i++) {
        if ((slutrutt[i] == h_u && slutrutt[i+1] == h_v) || (slutrutt[i] == h_v && slutrutt[i+1] == h_u)) {
            int nuvarande_nod = slutrutt[i];
            stang_av_vag(h_u, h_v);
            char riktning = nodriktningsmatris[nuvarande_nod][slutrutt[i+1]];
            uppdatera_fas2(nuvarande_nod, get_motsatt_riktning(riktning));
            printf(">>> HINDER %d-%d HITTTAT PÅ VÄG HEM! NY RUTT BERÄKNAD:\n", h_u, h_v);
            print_fas2(false);
            return;
        }
    }
}

// =================================================================
// MAIN
// =================================================================
int main() {
    init_karta();
    
    // --- KONFIGURATION ---
    mal_u = 16; 
    mal_v = 17; 
    char start_riktning = 's'; 
    
    // Ändra dessa för att testa hinder! (Sätt till NONE för att stänga av hindret)
    int hinder1_u = 10, hinder1_v = 15; // Hinder på ditvägen
    int hinder2_u = 18, hinder2_v = 19; // Hinder på hemvägen
    // ---------------------

    printf("=== INITIAL RUTT ===\n");
    uppdatera_fas1_och_2(START, start_riktning);
    print_fas1(); 
    print_fas2(true);

    kolla_och_applicera_hinder_fas1(hinder1_u, hinder1_v);
    kolla_och_applicera_hinder_fas2(hinder2_u, hinder2_v);

    return 0;
}