#include "pathfinding.h"
#include <string.h>
#include <stdbool.h>

// =================================================================
// GLOBALA VARIABLER (karta, rutter, varudata)
// =================================================================

char nodriktningsmatris[NODES][NODES];
int  vag[NODES][NODES];
int  rutt_till_vara[NODES];
int  rutt_hem[NODES];
char beslut_till_vara[NODES];
char beslut_hem[NODES];
int  vara_u, vara_v;

uint8_t item_list_u[MAX_ITEMS];
uint8_t item_list_v[MAX_ITEMS];
int item_count         = 0;
int current_item_index = 0;
int pickup_ingang, pickup_utgang;
char dir_vid_vara;

// =================================================================
// KARTA: Initiering av 5x5-rutnät + startnod
// =================================================================
void init_karta() {
    memset(vag, 0, sizeof(vag));
    memset(nodriktningsmatris, ' ', sizeof(nodriktningsmatris));

    for (int i = 0; i < 25; i++) {
        int rad = i / 5;
        int kol = i % 5;
        if (kol < 4) { vag[i][i+1] = 1; nodriktningsmatris[i][i+1] = 'e'; } // Kant österut
        if (kol > 0) { vag[i][i-1] = 1; nodriktningsmatris[i][i-1] = 'w'; } // Kant västerut
        if (rad < 4) { vag[i][i+5] = 1; nodriktningsmatris[i][i+5] = 's'; } // Kant söderut
        if (rad > 0) { vag[i][i-5] = 1; nodriktningsmatris[i][i-5] = 'n'; } // Kant norrut
    }

    // Koppla startnod (25) till nod 0 (övre vänstra hörnet)
    vag[START][0] = 1; 
    vag[0][START] = 1; 
    nodriktningsmatris[START][0] = 's';
    nodriktningsmatris[0][START] = 'n';
}

// =================================================================
// HJÄLPFUNKTIONER: Svängar och riktningar
// =================================================================
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
    beslut[i]   = 'X';
    beslut[i+1] = '\0';
}

// =================================================================
// DIJKSTRAS
// =================================================================
int hitta_rutt(int start, int mal, int rutt[], char start_dir) {
    int kostnad[NODES];
    int foregaende[NODES];
    char riktning_in[NODES];
    bool besokt[NODES] = {false};

    for (int i = 0; i < NODES; i++) {
        kostnad[i]    = 9999;
        foregaende[i] = NONE;
        rutt[i]       = STOP;
    }

    kostnad[start]     = 0;
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
                int straff     = (riktning_in[u] != nasta_dir) ? 1 : 0;
                int ny_kostnad = kostnad[u] + 100 + straff;
                if (ny_kostnad < kostnad[v]) {
                    kostnad[v]     = ny_kostnad;
                    foregaende[v]  = u;
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
// RUTTPLANERING
// =================================================================
void planera_till_vara(int from_node, char from_dir) {
    int rutt_alt1[NODES], rutt_alt2[NODES];

    int kostnad1 = hitta_rutt(from_node, vara_u, rutt_alt1, from_dir);
    int kostnad2 = hitta_rutt(from_node, vara_v, rutt_alt2, from_dir);

    if (kostnad1 <= kostnad2) {
        pickup_ingang = vara_u; pickup_utgang = vara_v;
        memcpy(rutt_till_vara, rutt_alt1, sizeof(rutt_alt1));
    } else {
        pickup_ingang = vara_v; pickup_utgang = vara_u;
        memcpy(rutt_till_vara, rutt_alt2, sizeof(rutt_alt2));
    }

    int i = 0;
    while (rutt_till_vara[i] != STOP) i++;
    rutt_till_vara[i]   = pickup_utgang;
    rutt_till_vara[i+1] = STOP;

    bygg_beslut(rutt_till_vara, from_dir, beslut_till_vara);
    dir_vid_vara = nodriktningsmatris[pickup_ingang][pickup_utgang];
}

void planera_hem_fran_pickup() {
    int rutt_alt1[NODES], rutt_alt2[NODES];

    int cost_fwd  = hitta_rutt(pickup_utgang, START, rutt_alt1, dir_vid_vara);
    // Vid backup: roboten backar till pickup_ingang men tittar fortfarande åt dir_vid_vara
    int cost_back = hitta_rutt(pickup_ingang, START, rutt_alt2, dir_vid_vara) + 100;

    if (cost_fwd <= cost_back) {
        memcpy(rutt_hem, rutt_alt1, sizeof(rutt_alt1));
        bygg_beslut(rutt_hem, dir_vid_vara, beslut_hem);
        int dlen = strlen(beslut_hem) + 1;
        memmove(&beslut_hem[1], &beslut_hem[0], dlen);
        beslut_hem[0] = 'f';
        int rlen = 0; while (rutt_hem[rlen] != STOP) rlen++;
        memmove(&rutt_hem[1], &rutt_hem[0], (rlen + 1) * sizeof(int));
        rutt_hem[0] = pickup_ingang;
    } else {
        memcpy(rutt_hem, rutt_alt2, sizeof(rutt_alt2));
        bygg_beslut(rutt_hem, dir_vid_vara, beslut_hem);
        int dlen = strlen(beslut_hem) + 1;
        memmove(&beslut_hem[1], &beslut_hem[0], dlen);
        beslut_hem[0] = 'b';
        int rlen = 0; while (rutt_hem[rlen] != STOP) rlen++;
        memmove(&rutt_hem[1], &rutt_hem[0], (rlen + 1) * sizeof(int));
        rutt_hem[0] = pickup_utgang;
    }
}

void planera_nasta_vara() {
    int rutt_tmp[NODES];

    int costs[4];
    costs[0] = hitta_rutt(pickup_utgang, vara_u, rutt_tmp, dir_vid_vara);
    costs[1] = hitta_rutt(pickup_utgang, vara_v, rutt_tmp, dir_vid_vara);
    // Vid backup: roboten tittar fortfarande åt dir_vid_vara från pickup_ingang
    costs[2] = hitta_rutt(pickup_ingang, vara_u, rutt_tmp, dir_vid_vara) + 100;
    costs[3] = hitta_rutt(pickup_ingang, vara_v, rutt_tmp, dir_vid_vara) + 100;

    int best = 0;
    for (int i = 1; i < 4; i++) { if (costs[i] < costs[best]) best = i; }

    bool backup = (best >= 2);
    int from_node = backup ? pickup_ingang : pickup_utgang;
    char from_dir = dir_vid_vara; // Samma riktning oavsett backup eller framåt
    int approach  = (best % 2 == 0) ? vara_u : vara_v;
    int through   = (approach == vara_u) ? vara_v : vara_u;

    hitta_rutt(from_node, approach, rutt_till_vara, from_dir);
    int i = 0;
    while (rutt_till_vara[i] != STOP) i++;
    rutt_till_vara[i]   = through;
    rutt_till_vara[i+1] = STOP;

    bygg_beslut(rutt_till_vara, from_dir, beslut_till_vara);

    int dlen = strlen(beslut_till_vara) + 1;
    memmove(&beslut_till_vara[1], &beslut_till_vara[0], dlen);
    beslut_till_vara[0] = backup ? 'b' : 'f';

    int rlen = 0; while (rutt_till_vara[rlen] != STOP) rlen++;
    memmove(&rutt_till_vara[1], &rutt_till_vara[0], (rlen + 1) * sizeof(int));
    rutt_till_vara[0] = backup ? pickup_utgang : pickup_ingang;

    pickup_ingang = approach;
    pickup_utgang = through;
    dir_vid_vara  = nodriktningsmatris[approach][through];
}
