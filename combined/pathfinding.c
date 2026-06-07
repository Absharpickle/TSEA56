//-------------------------------------------------------
// Markus Hellers, Joel Eberhardsson - 28 Maj 2026 - V1.0
//-------------------------------------------------------

#include "pathfinding.h"
#include <string.h>
#include <stdbool.h>

char nodriktningsmatris[NODES][NODES];      //väderstreck för varje väg 'e' 'w' 'n' 's'
int  vag[NODES][NODES];                     //om en väg mellan två noder existerar/tillåten, == 0/1
int  rutt_till_vara[NODES];                 //rutt till vara i form av nodarray [25, 0, 1, 2, ..., STOP]
int  rutt_hem[NODES];                       //rutt till hem i form av nodarray [..., 2, 1, 0, 25, STOP]
char beslut_till_vara[NODES];               //beslut till till vara i form av beslutsarray ['f', 'o', 'f', ..., 'X']
char beslut_hem[NODES];                     //beslut till till hem i form av beslutsarray ['b', ..., 'e', 'o', 'X']

int  vara_u, vara_v;                        //noder mellan varan. anänds för att sätta ut varan.
uint8_t item_list_u[MAX_ITEMS];             //del-lista för alla noder mellan vara
uint8_t item_list_v[MAX_ITEMS];             //del-lista för alla noder mellan vara
int item_count         = 0;                 //hur många varor på banan, börja med 0 varor
int current_item_index = 0;                 //vilken vara vi hämtar just nu, börja med ingen vara
int pickup_ingang, pickup_utgang;           //vilken nod vi ska köra in/köra ut när vi hämtar varan
char dir_vid_vara;                          //väderstreck vid varan

//initierar kartan (tilldelar värden i nodriktningsmatris[][] & vag[][])
void init_karta() {
    memset(vag, 0, sizeof(vag));    //snabbfunktion, sätter alla vägar till inaktiva
    memset(nodriktningsmatris, ' ', sizeof(nodriktningsmatris));     //snabbfunktion, sätter alla väderstreck till blankt

    for (int i = 0; i < NODES - 1; i++) {
        int rad = i / 5;        //raden är alltid noden dividerat med 5 avrundat nedåt t.ex. nod 5,6,7,8,9 ligger på rad 1
        int kol = i % 5;        //kolumnen är alltid noden modulo 5 dvs nod 6, 11, 16, 21 ligger på kolumn 1
        
        if (kol < 4) {                               //finns inga vägar öster om kolumn 4
            vag[i][i+1] = 1;                         //vägar öster om en nod är nod + 1
            nodriktningsmatris[i][i+1] = 'e'; }      //väg österut
        if (kol > 0) {                               //finns inga vägar väster om kolumn 0
            vag[i][i-1] = 1;                         //vägar öster om en nod är nod - 1
            nodriktningsmatris[i][i-1] = 'w'; }      //väg västerut
        if (rad < 4) {                               //finns inga vägar söder om rad 4
            vag[i][i+5] = 1;                         //vägar öster om en nod är nod + 5
            nodriktningsmatris[i][i+5] = 's'; }      //väg söderut
        if (rad > 0) {                               //finns inga vägar norr om rad 0
            vag[i][i-5] = 1;                         //vägar öster om en nod är nod - 5
            nodriktningsmatris[i][i-5] = 'n'; }      //väg norrut
    }

    //aktivera väg mellan startnod (25) och nod 0 (nod 25 ligger norr om nod 0)
    vag[START][0] = 1; 
    vag[0][START] = 1; 
    nodriktningsmatris[START][0] = 's';
    nodriktningsmatris[0][START] = 'n';
}

//hjälpfunktion för att översätta väderstreck till beslut
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
    return 'f'; //vid fel returnera 'f'
}

//ger motsatt väderstreck
char get_motsatt_dir(char nu) {
    if (nu == 's') return 'n';
    if (nu == 'n') return 's';
    if (nu == 'e') return 'w';
    if (nu == 'w') return 'e';
    return nu;
}

//dijkstras algoritm för ruttarray
int hitta_rutt(int start, int mal, int rutt[], char start_dir) {
    int kostnad[NODES];             //array för noders kostnad
    int foregaende[NODES];          //array för noders föregående nod
    char riktning_in[NODES];        //array för väderstreck in i en nod från en annan nod
    bool besokt[NODES] = {false};   //array för om nod är besökt i algoritmen, false betyder ej besökt

    //nollställ arrayer
    for (int i = 0; i < NODES; i++) {
        kostnad[i]    = 9999;       //oändlig kostnad i början för alla noder       
        foregaende[i] = NONE;       //noder har ingen föregående i början av sökning
        rutt[i]       = STOP;       //sätter stopvillkor i början av ruttarrayen, då arrayen kommer att vändas och villkoret hamnar i slutet
    }

    kostnad[start]     = 0;              //kostnaden för utgångsnoden i sökningen sätts till noll
    riktning_in[start] = start_dir;      //ingången in i första noden sätts till riktningen roboten har i tillfället

    for (int i = 0; i < NODES; i++) {    //iterar sökningen 26 (max antal noder) gånger
        int u = -1;                      //u är noden vi avsöker, -1 behövs för att sätta igång sökningen
       
        for (int j = 0; j < NODES; j++) { //itererar alla noder för att hitta nästa vi ska söka av
            //om kostanden för en nod är lägre än den hittils lägsta, uppdaterar vi nod som skall avsökas     
            //i början är alla kostnader 9999 (förutom startnoden) och då behövs villkoret u == -1 för att hitta startnoden
            if (!besokt[j] && (u == -1 || kostnad[j] < kostnad[u])){
                u = j; 
            }
        }
        if (kostnad[u] == 9999 || u == mal){
            break; //om lägsta kostnaden vi avsöker är oändlig (ingen väg till noden) eller vi har hittat slutnoden avbryter vi sök
        }
        besokt[u] = true;   //sätter noden som vi ska söka av till besökt

        for (int v = 0; v < NODES; v++) {                   //itererar alla noder för att hitta grannar
            if (vag[u][v] && !besokt[v]) {                  //om noden är en granne och ej besökt
                char nasta_dir = nodriktningsmatris[u][v];  //hitta riktning in till grannen
                int straff = 0;                             //deklarera straffvariabel och sätt till noll
               
                if (riktning_in[u] != nasta_dir) { //om vi byter riktning ska applicera straff (sväng eller backning)
                    if (nasta_dir == get_motsatt_dir(riktning_in[u])){ 
                        straff = 9999; //motverka backning
                    }else{
                        straff = 1; //straffa med 1 om roboten skall svänga
                    }
                }

                int ny_kostnad = kostnad[u] + 100 + straff; //temp kostnad för nod v
                if (ny_kostnad < kostnad[v]) {  //om temp är hittils lägsta
                    kostnad[v]     = ny_kostnad; //uppdatera kostnad
                    foregaende[v]  = u;          //sätt föregående nod til u
                    riktning_in[v] = nasta_dir;  //sätt riktning in till nod v från u
                }
            }
        }
    }

    //procedur  vända på ruttarrayen

    int temp[NODES]; //temp array med storlek antal noder
    int c = 0;       //index som i slutet blir arrayens storlek
    int nu = mal;    //nod vi tilldelar arrayen just nu (börja med slutnod)

    while (nu != NONE) {        //tilldela värden tills vi hittar stoppvillkor i arrayen
        temp[c++] = nu;         //tilldela värdet (första är mål) och sedan uppdatera index
        nu = foregaende[nu];    //uppatera nod till föregående
    }

    for (int i = 0; i < c; i++){
        rutt[i] = temp[c - 1 - i];      //vänd på hela arrayen
    }
    return kostnad[mal];                //returnera kostnad för rutten (för att jämföra olika ingångar (slutnoder) till vara)
}


//översätter ruttarray till beslutsarray beroende på var roboten startar
void bygg_beslut(int rutt[], char start_dir, char beslut[]) {
    char dir = start_dir;     //ansätt startriktning         
    int i = 0;                //index

    while (rutt[i + 1] != STOP) { //iterera tills stoppvillkoret i ruttarray är uppnådd
        char nasta_dir = nodriktningsmatris[rutt[i]][rutt[i + 1]];  //hitta riktning mellan nod och nästa nod i ruttarrayen
        beslut[i] = get_turn(dir, nasta_dir);                       //översätt riktningsförhållandet till ett beslut
        dir = nasta_dir;                                            //uppdatera riktning
        i++;                                                        //uppdatera index
    }
    beslut[i]   = 'X';      //sätt sista beslutet till 'X' (plocka, lämna)           
    beslut[i+1] = '\0';     //karaktär för avsluta arrayen (avlustar sträng)
}


//bestämmer bästa beslutsarray till varan
void planera_till_vara(int from_node, char from_dir) {
    int rutt_alt1[NODES], rutt_alt2[NODES]; //temporära ruttvariabler för jämförelse

    int kostnad1 = hitta_rutt(from_node, vara_u, rutt_alt1, from_dir);  //reurnerar kostand för ingångsnod v, tilldelar värden till rutt_alt1
    int kostnad2 = hitta_rutt(from_node, vara_v, rutt_alt2, from_dir);  //reurnerar kostand för ingångsnod u, tilldelar värden till rutt_alt2

    if (kostnad1 <= kostnad2) {     //jämför kostander för att avgöra vilken rutt som är bäst
        pickup_ingang = vara_u;     //uppdaterar ingång
        pickup_utgang = vara_v;     //uppdaterar utgång
        memcpy(rutt_till_vara, rutt_alt1, sizeof(rutt_alt1)); //snabbfunktion, tilldelar värden till rutt_till_vara 
    } else {
        pickup_ingang = vara_v;     //uppdaterar ingång
        pickup_utgang = vara_u;     //uppdaterar utgång
        memcpy(rutt_till_vara, rutt_alt2, sizeof(rutt_alt2)); //snabbfunktion, tilldelar värden till rutt_till_vara 
    }

    int i = 0;
    while (rutt_till_vara[i] != STOP){ //hittar sista index
        i++;
    }
    rutt_till_vara[i]   = pickup_utgang; //sätter sista index till utgångsnod så att roboten hur den ska åka till varan
    rutt_till_vara[i+1] = STOP; 

    bygg_beslut(rutt_till_vara, from_dir, beslut_till_vara);    //översätter rutt till beslut
    dir_vid_vara = nodriktningsmatris[pickup_ingang][pickup_utgang];    //uppdaterar riktningen vid varan
}

//bestämmer bästa beslutsarray till hem
void planera_hem_fran_pickup(int from_node, char from_dir) {
    int rutt_alt1[NODES], rutt_alt2[NODES];
    int cost_fwd;   //temporära ruttvariabler för jämförelse
    int cost_back;  //temporära ruttvariabler för jämförelse

    if (from_node == 99) {      //99 är standardargument om man kör från vara
        cost_fwd  = hitta_rutt(pickup_utgang, START, rutt_alt1, dir_vid_vara);  //kostnad av att åka rakt fram efter hämtad vara
        cost_back = hitta_rutt(pickup_ingang, START, rutt_alt2, dir_vid_vara) + 100;   //kostnad av att backa efter hämtad vara
    } else {        
        cost_fwd  = hitta_rutt(from_node, START, rutt_alt1, from_dir);  //om hinder upptäckts
        cost_back = 9999;      //vill ej backa när upptäckt hinder då hindret är en väg fram från nästa nod
    }

    if (cost_fwd <= cost_back) {    //jämför kostnad för att åka fram eller bak från vara
        memcpy(rutt_hem, rutt_alt1, sizeof(rutt_alt1));     //snabbfunktion, för att kopiera temp till rutt_hem

        if (from_node == 99) {
            bygg_beslut(rutt_hem, dir_vid_vara, beslut_hem); //standargument från vara
        }else {
            bygg_beslut(rutt_hem, from_dir, beslut_hem); //från annanstans
        }
        if (from_node == 99) {  
            int dlen = strlen(beslut_hem) + 1;      //skifta så att 'f' hamnar först i beslut från vara då den inte har ett aktuellet kommando
            memmove(&beslut_hem[1], &beslut_hem[0], dlen);
            beslut_hem[0] = 'f';
            int rlen = 0; while (rutt_hem[rlen] != STOP) rlen++;
            memmove(&rutt_hem[1], &rutt_hem[0], (rlen + 1) * sizeof(int));
            rutt_hem[0] = pickup_ingang;
            }
    }else { 
        memcpy(rutt_hem, rutt_alt2, sizeof(rutt_alt2)); //vi ska använda rutten som backar
        bygg_beslut(rutt_hem, dir_vid_vara, beslut_hem);
        int dlen = strlen(beslut_hem) + 1;     //skifta så att 'b' hamnar först i beslut från vara då den inte har ett aktuellet kommando
        memmove(&beslut_hem[1], &beslut_hem[0], dlen);
        beslut_hem[0] = 'b'; 
        int rlen = 0; while (rutt_hem[rlen] != STOP) rlen++;
        memmove(&rutt_hem[1], &rutt_hem[0], (rlen + 1) * sizeof(int));
        rutt_hem[0] = pickup_utgang;
    }
}

//bestämmer bästa beslutsarray mellan varor
void planera_nasta_vara(int from_node1, char from_dir1) {
    int rutt_tmp[NODES];   
    int costs[4]; //fyra vägar existerar då man kör vara till vara eftersom varje vara har två utgångar/ingångar
    costs[0] = hitta_rutt(pickup_utgang, vara_u, rutt_tmp, dir_vid_vara);
    costs[1] = hitta_rutt(pickup_utgang, vara_v, rutt_tmp, dir_vid_vara);
    costs[2] = hitta_rutt(pickup_ingang, vara_u, rutt_tmp, dir_vid_vara) + 100; //backning
    costs[3] = hitta_rutt(pickup_ingang, vara_v, rutt_tmp, dir_vid_vara) + 100; //backning

    if (from_node1 != 99) {     //om det upptäcks ett hinder 
        costs[0] = hitta_rutt(from_node1, vara_u, rutt_tmp, from_dir1); //förutsätt att roboten fortsätter åka rakt fram men kollar på vilken ingång till varan som går snabbast
        costs[1] = hitta_rutt(from_node1, vara_v, rutt_tmp, from_dir1);
        costs[2] = 9999; //ingen backning
        costs[3] = 9999; //ingen backning
    }

    int best = 0;
    for (int i = 1; i < 4; i++) {       //hitta bästa kostnad
        if (costs[i] < costs[best]) best = i;
    }

    bool backup = (best >= 2); //om bästa kostanden är index 2 eller 3 så behöver vi backa

    int from_node;  
    if (backup) {
        from_node = pickup_ingang;  //åk ut där roboten kom från
    } else {
        from_node = pickup_utgang;  //åk ut på andra sidan
    }

    char from_dir = dir_vid_vara;   

    int approach;
    if (best % 2 == 0) {    //vilken nod ska vi åka in i
        approach = vara_u;  
    } else {
        approach = vara_v;
    }

    int through;
    if (approach == vara_u) { //vilken nod ska vi åka mot (inte nödväntigtvis ut)
        through = vara_v;
    } else {
        through = vara_u;
    }

    if (from_node1 != 99) { //vid hinder
        from_node = from_node1;
        from_dir  = from_dir1;
    }

    hitta_rutt(from_node, approach, rutt_till_vara, from_dir);
    int i = 0;
    while (rutt_till_vara[i] != STOP) i++;
    rutt_till_vara[i]   = through; //sista nod är noden vi åker mot när varan skall hämtas
    rutt_till_vara[i+1] = STOP;

    bygg_beslut(rutt_till_vara, from_dir, beslut_till_vara); 

    if (from_node1 == 99) { //prodedur för att lägga ett 'b' eller 'f' i början av beslutsarrayen
        int dlen = strlen(beslut_till_vara) + 1;
        memmove(&beslut_till_vara[1], &beslut_till_vara[0], dlen);
        if (backup) {
            beslut_till_vara[0] = 'b'; 
        } else {
            beslut_till_vara[0] = 'f';
        }

        int rlen = 0;
        while (rutt_till_vara[rlen] != STOP) rlen++;
        memmove(&rutt_till_vara[1], &rutt_till_vara[0], (rlen + 1) * sizeof(int));
        if (backup) {
            rutt_till_vara[0] = pickup_utgang;
        } else {
            rutt_till_vara[0] = pickup_ingang;
        }
    }

    pickup_ingang = approach;
    pickup_utgang = through;
    dir_vid_vara  = nodriktningsmatris[approach][through];
}