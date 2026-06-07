//-------------------------------------------------------
// Markus Hellers, Joel Eberhardsson - 28 Maj 2026 - V1.0
//-------------------------------------------------------

#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <stdbool.h>
#include <stdint.h>

#define NODES 26        //5x5 noder plus en startnod. Detta värde kan ändras för att omforma banan.
#define START 25        //start/slut-nod tilldelas värdet 25 då 0-24 är upptagna
#define NONE -1         //används som odefinierat värde när det inte finns en föregående nod i algoritmen
#define STOP -1         //stoppvillkor i ruttarrayen
#define MAX_ITEMS 5     //max antal varor roboten kan plocka

extern char nodriktningsmatris[NODES][NODES];       //väderstreck för varje väg 'e' 'w' 'n' 's'
extern int  vag[NODES][NODES];                      //om en väg mellan två noder existerar/tillåten, == 0/1
extern int  rutt_till_vara[NODES];                  //rutt till vara i form av nodarray [25, 0, 1, 2, ..., STOP]
extern int  rutt_hem[NODES];                        //rutt till hem i form av nodarray [..., 2, 1, 0, 25, STOP]
extern char beslut_till_vara[NODES];                //beslut till till vara i form av beslutsarray ['f', 'o', 'f', ..., 'X']
extern char beslut_hem[NODES];                      //beslut till till hem i form av beslutsarray ['b', ..., 'e', 'o', 'X']

extern int  vara_u, vara_v;                         //noder mellan varan. anänds för att sätta ut varan.
extern uint8_t item_list_u[MAX_ITEMS];              //del-lista för alla noder mellan vara
extern uint8_t item_list_v[MAX_ITEMS];              //del-lista för alla noder mellan vara
extern int item_count;                              //hur många varor på banan
extern int current_item_index;                      //vilken vara vi hämtar just nu
extern int pickup_ingang, pickup_utgang;            //vilken nod vi ska köra in/köra ut när vi hämtar varan
extern char dir_vid_vara;                           //väderstreck vid varan


void init_karta();                                                  //initierar kartan (tilldelar värden i nodriktningsmatris[][] & vag[][])             
char get_turn(char nu, char nasta);                                 //hjälpfunktion för att översätta väderstreck till beslut
char get_motsatt_dir(char nu);                                      //ger motsatt väderstreck
int  hitta_rutt(int start, int mal, int rutt[], char start_dir);    //dijkstras algoritm för ruttarray
void bygg_beslut(int rutt[], char start_dir, char beslut[]);        //översätter ruttarray till beslutsarray beroende på var roboten startar
void planera_till_vara(int from_node, char from_dir);               //bestämmer bästa beslutsarray till varan
void planera_hem_fran_pickup(int from_node, char from_dir);         //bestämmer bästa beslutsarray till hem
void planera_nasta_vara(int from_node1, char from_dir1);            //bestämmer bästa beslutsarray mellan varor

#endif 