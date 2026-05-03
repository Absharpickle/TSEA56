#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <stdbool.h>
#include <stdint.h>

// --- ALGORITHM DEFINITIONS ---
#define NODES 26     // 5x5 samt en start/slutnod
#define START 25     // Start/slut på nod 25
#define NONE -1      // Betyder att det inte finns en föregående nod
#define STOP -1      // Stoppvillkor för ruttarray
#define MAX_ITEMS 20 // Max antal varor per körning

// --- MAP DATA (definierade i pathfinding.c) ---
extern char nodriktningsmatris[NODES][NODES];
extern int  vag[NODES][NODES];

// --- ROUTE ARRAYS ---
extern int  rutt_till_vara[NODES];
extern int  rutt_hem[NODES];
extern char beslut_till_vara[NODES];
extern char beslut_hem[NODES];

// --- ITEM STATE ---
extern int  vara_u, vara_v;
extern uint8_t item_list_u[MAX_ITEMS];
extern uint8_t item_list_v[MAX_ITEMS];
extern int item_count;
extern int current_item_index;
extern int pickup_ingang, pickup_utgang;
extern char dir_vid_vara;

// --- FUNCTIONS ---
void init_karta();
char get_turn(char nu, char nasta);
char get_motsatt_dir(char nu);
void bygg_beslut(int rutt[], char start_dir, char beslut[]);
int  hitta_rutt(int start, int mal, int rutt[], char start_dir);
void planera_till_vara(int from_node, char from_dir);
void planera_hem_fran_pickup();
void planera_nasta_vara();

#endif // PATHFINDING_H
