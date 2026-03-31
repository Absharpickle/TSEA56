#include <stdio.h>
#include <stdbool.h>

#define NUM_NODES 27 
#define INF -1

// Färg-konstanter för BFS
#define WHITE 0
#define GRAY 1
#define BLACK 2

// --- GLOBAL DATA FRÅN DESIGN SPEC 4.4.1 ---
char nodriktningsmatris[NUM_NODES][NUM_NODES];
int graph[NUM_NODES][NUM_NODES]; 

int malrutt[NUM_NODES];     
char nodbeslut[NUM_NODES];  

// Funktion för att bygga kartan enligt 5x5-nätet i specifikationen
void init_karta() {
    // Nollställ matriser
    for(int i=0; i<NUM_NODES; i++) {
        for(int j=0; j<NUM_NODES; j++) {
            graph[i][j] = 0;
            nodriktningsmatris[i][j] = ' ';
        }
    }

    // Skapa 5x5 grid (Nod 0 till 24)
    for (int i = 0; i < 25; i++) {
        int rad = i / 5;
        int kol = i % 5;

        // Granne till höger (Öst)
        if (kol < 4) {
            graph[i][i + 1] = 1;
            nodriktningsmatris[i][i + 1] = 'e';
        }
        // Granne till vänster (Väst)
        if (kol > 0) {
            graph[i][i - 1] = 1;
            nodriktningsmatris[i][i - 1] = 'w';
        }
        // Granne nedåt (Syd)
        if (rad < 4) {
            graph[i][i + 5] = 1;
            nodriktningsmatris[i][i + 5] = 's';
        }
        // Granne uppåt (Norr)
        if (rad > 0) {
            graph[i][i - 5] = 1;
            nodriktningsmatris[i][i - 5] = 'n';
        }
    }

    // Lägg till de extra noderna (25 = Start, 26 = Slut)
    // Exempel: Startnod 25 ansluter till nod 0
    graph[25][0] = 1; nodriktningsmatris[25][0] = 'e'; 
    // Exempel: Nod 24 ansluter till slutnod 26
    graph[24][26] = 1; nodriktningsmatris[24][26] = 'e';
}

void optimeringsfunktion(int startnod, int slutnod, int rutt_array[]) {
    int color[NUM_NODES], d[NUM_NODES], pi[NUM_NODES];
    int queue[NUM_NODES], head = 0, tail = 0;

    for (int u = 0; u < NUM_NODES; u++) {
        color[u] = WHITE; d[u] = INF; pi[u] = -1;
        rutt_array[u] = -1;
    }

    color[startnod] = GRAY; d[startnod] = 0;
    queue[tail++] = startnod;

    while (head < tail) {
        int u = queue[head++];
        if (u == slutnod) break;

        for (int v = 0; v < NUM_NODES; v++) {
            if (graph[u][v] == 1 && color[v] == WHITE) {
                color[v] = GRAY;
                d[v] = d[u] + 1;
                pi[v] = u;
                queue[tail++] = v;
            }
        }
        color[u] = BLACK;
    }

    // Rekonstruera vägen
    int temp_rutt[NUM_NODES];
    int curr = slutnod, count = 0;
    while (curr != -1) {
        temp_rutt[count++] = curr;
        curr = pi[curr];
    }
    for (int i = 0; i < count; i++) {
        rutt_array[i] = temp_rutt[count - 1 - i];
    }
}

// Utökad beslutsfunktion som tar hänsyn till robotens riktning
void beslutsfunktion(int rutt_array[], char start_riktning) {
    char nuvarande_riktning = start_riktning;

    for (int i = 0; rutt_array[i+1] != -1; i++) {
        int u = rutt_array[i];
        int v = rutt_array[i+1];
        char mal_riktning = nodriktningsmatris[u][v];

        // Logik för att översätta väderstreck till robot-kommandon
        if (nuvarande_riktning == mal_riktning) {
            nodbeslut[i] = 'f'; // Samma riktning -> Framåt
        } else {
            // Här kan man lägga till logik för 'r' (höger), 'l' (vänster), 'b' (bakåt)
            // Exempel: Om vi kollar Norr men ska Österut -> Sväng Höger
            if ((nuvarande_riktning == 'n' && mal_riktning == 'e') ||
                (nuvarande_riktning == 'e' && mal_riktning == 's') ||
                (nuvarande_riktning == 's' && mal_riktning == 'w') ||
                (nuvarande_riktning == 'w' && mal_riktning == 'n')) {
                nodbeslut[i] = 'r'; 
            } else {
                nodbeslut[i] = 'l'; 
            }
        }
        nuvarande_riktning = mal_riktning; // Uppdatera robotens kompass
    }
}

int main() {
    init_karta(); // Bygg miljön först!

    // TEST: Gå från nod 1 till nod 5
    // (I en 5x5 grid är nod 1 och 5 grannar vertikalt)
    int start = 1;
    int mal = 9;

    optimeringsfunktion(start, mal, malrutt);

    printf("Snabbaste rutten från %d till %d: ", start, mal);
    for (int i = 0; malrutt[i] != -1; i++) {
        printf("%d ", malrutt[i]);
    }
    printf("\n");

    // Generera beslut (anta att roboten startar vänd mot Syd)
    beslutsfunktion(malrutt, 's');

    printf("Beslutssekvens: ");
    for (int i = 0; malrutt[i+1] != -1; i++) {
        printf("%c ", nodbeslut[i]);
    }
    printf("\n");

    return 0;
}