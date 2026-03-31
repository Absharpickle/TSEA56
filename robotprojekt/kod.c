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

char nodriktningsmatris[NODES][NODES];
int vag[NODES][NODES];
int malrutt[NODES];
char nodbeslut[NODES];

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
    vag[START][0] = 1;
    vag[0][START] = 1;
    nodriktningsmatris[START][0] = 's'; //ej bestämt?
    nodriktningsmatris[0][START] = 'n';
    vag[24][END] = 1;
    vag[END][24] = 1;
    nodriktningsmatris[24][END] = 's'; //ej bestämt?
    nodriktningsmatris[END][24] = 'n';
}

void optimeringsfunktion(int startnod, int slutnod, int rutt_array[]){
    int color[NODES];
    int distance[NODES];
    int prev[NODES];
    int queue[NODES];
    int first = 0;
    int last = 0;

    for (int u = 0; u < NODES; u++){
        color[u] = WHITE;
        distance[u] = INF;
        prev[u] = NONE;
        rutt_array[u] = STOP;
    }

    color[startnod] = GRAY;
    distance[startnod] = 0;
    queue[last] = startnod;
    last++;

    while (first < last) {
        int u = queue[first];
        first++;
        if (u == slutnod) break;

        for (int v = 0; v < NODES; v++){
            if (vag[u][v] == 1 && color[v] == WHITE){
                color[v] = GRAY;
                distance[v] = distance[u] + 1;
                prev[v] = u;
                queue[last] = v;
                last++;
            }
        }
        color[u] = BLACK;
    }

    int temp_rutt[NODES];
    int current = slutnod;
    int count = 0;

    while (current != NONE){
        temp_rutt[count] = current;
        current = prev[current];
        count++;
    }
    for (int i = 0; i < count; i++){
        rutt_array[i] = temp_rutt[count - 1 - i];
    }
}

void beslutsfunktion(int rutt_array[], char start_riktning) {
    char nuvarande_riktning = start_riktning;

    for (int i = 0; rutt_array[i + 1] != STOP; i++){
        int u = rutt_array[i];
        int v = rutt_array[i + 1];
        char mal_riktning = nodriktningsmatris[u][v];
        
        if (nuvarande_riktning == mal_riktning){
            nodbeslut[i] = 'f';
        } else {
            if ((nuvarande_riktning == 'n' && mal_riktning == 'e') ||
                (nuvarande_riktning == 'e' && mal_riktning == 's') ||
                (nuvarande_riktning == 's' && mal_riktning == 'w') ||
                (nuvarande_riktning == 'w' && mal_riktning == 'n')) {
                nodbeslut[i] = 'r';
            } else {
                if ((nuvarande_riktning == 'n' && mal_riktning == 'w') ||
                    (nuvarande_riktning == 'w' && mal_riktning == 's') ||
                    (nuvarande_riktning == 's' && mal_riktning == 'e') ||
                    (nuvarande_riktning == 'e' && mal_riktning == 'n')) {
                    nodbeslut[i] = 'l';
                } else {
                    nodbeslut[i] = 'b';
                }
            }
        }
        nuvarande_riktning = mal_riktning; 
    }
}
int main(){
    init_karta();
    int start = 1;
    int mal = END;
    optimeringsfunktion(start, mal, malrutt);

    printf("Snabbaste rutten från %d till %d: ", start, mal);
    for (int i = 0; malrutt[i] != STOP; i++) {
        printf("%d ", malrutt[i]);
    }
    printf("\n");

    beslutsfunktion(malrutt, 's');
    
    printf("Beslutssekvens: ");
    for (int i = 0; malrutt[i + 1] != STOP; i++){
        printf("%c ", nodbeslut[i]);
    }
    printf("\n");
    return 0;
}