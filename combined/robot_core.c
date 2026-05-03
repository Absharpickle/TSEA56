#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <time.h>
#include <stdint.h> 

// --- ALGORITHM DEFINITIONS ---
#define NODES 26 // 5x5 samt en start/slutnod
#define START 25 // Start/slut på nod 25
#define NONE -1  // Betyder att det inte finns en föregående nod
#define STOP -1  // Stoppvillkor för ruttarray
#define MAX_ITEMS 20 // Max antal varor per körning

// --- TELEMETRY DEFINITIONS ---
#define UDP_PORT 5001           // Porten som används för kommunikation med persondatorn
#define BUFFER_SIZE 1024        // Storleken på bufferten
#define I2C_DEVICE "/dev/i2c-1" // Filnamn för i2c-bussen
#define STYRKOMM_ADDR 0x12
#define SENSOR_ADDR   0x10
#define PACKET_SIZE 8           // Paketstorleken för kommunikationen inom systemet (i2c)
#define VERIFY_LOG_FILE "verifikation_keys.txt" // Loggfil

// --- SIM MODE DEFINITIONS ---
#define SIM_SEGMENT_MS 3000     // Simulerad tid (ms) mellan korsningar i sim-läge

// --- ALGORITHM GLOBALS ---
char nodriktningsmatris[NODES][NODES]; // Riktning mellan noder: 'n','s','e','w'
int  vag[NODES][NODES];                // Grannmatris: 1 = kant finns, 0 = ingen kant
int  rutt_till_vara[NODES];            // Nodnummer-sekvens fram till varan
int  rutt_hem[NODES];                  // Nodnummer-sekvens hem från varan
char beslut_till_vara[NODES];          // Beslutslista (f/e/o/u/X) för varuresa
char beslut_hem[NODES];                // Beslutslista (f/e/o/u/X) för hemresa
int  vara_u, vara_v;                   // De två noderna som varan befinner sig mellan

// --- MULTI-ITEM GLOBALS ---
uint8_t item_list_u[MAX_ITEMS];  // Nod U för varje vara
uint8_t item_list_v[MAX_ITEMS];  // Nod V för varje vara
int item_count         = 0;      // Antal varor mottagna från GUI
int current_item_index = 0;      // Vilken vara vi jobbar med just nu (0-baserat)
int pickup_ingang, pickup_utgang; // Ingångs-/utgångsnod vid senaste pickup
char dir_vid_vara;                // Riktning vid varan (ingang→utgang)

// --- STATE MACHINE GLOBALS ---
typedef enum {
    PHASE_IDLE = 0,  // Väntar på kommando
    PHASE_TO_ITEM,   // Kör mot varan
    PHASE_PICKUP,    // Stannar och plockar upp varan
    PHASE_TO_HOME    // Kör hem
} AutoPhase;

AutoPhase current_phase      = PHASE_IDLE; // Aktuell fas i det autonoma körprogrammet
int current_action_index     = 0;          // Index i beslutslistan för nuvarande fas
unsigned char current_auto_state = 1;      // Vilket körläge som skickades med start-kommandot
bool log_next_action         = false;      // Flagga: skriv ut nästa beslutsbyte till konsolen

bool is_rotating   = false; // Sant medan roboten svänger på plats (tidsstyrd)
bool is_picking_up = false; // Sant medan roboten utför pickup-sekvensen (tidsstyrd)
long long action_timer_start = 0; // Tidpunkt (i millisekunder) då nuvarande tidsstyrd åtgärd startades
uint8_t korsning_aktiv = 0;       // Debounce: 1 = vi är inne på en korsning/markering just nu

// --- SIM MODE GLOBALS ---
bool sim_sensor = false; // Sant om sensorkortet (0x10) saknas → simulera korsningar med timer
bool sim_motor  = false; // Sant om motorstyrningen (0x12) saknas → hoppa över I2C-skrivningar
long long sim_segment_timer = 0; // Tidpunkt då vi börjar vänta på nästa simulerade korsning

// --- TELEMETRY GLOBALS FÖR GUI ---
bool gui_known        = false; // Sant när vi fått minst ett paket från GUI:n (vet IP/port)
int telemetry_counter = 0;     // Räknar loop-iterationer för 10 Hz telemetri-throttle

// --- LOOP TIMING GLOBALS ---
char nasta_beslut  = 's'; // Nästa beslut i kön (index+1) – visas på GUI som förhandsvisning
char aktivt_beslut = 's'; // Det beslut som just nu skickas till motorstyrningen
int  loop_counter  = 0;   // Generell loopräknare (används vid debug/timing)
uint8_t current_node = START; // Vilken nod roboten befinner sig vid (skickas i telemetri till GUI)

// =================================================================
// HJÄLPFUNKTION: Tidsmätning i millisekunder
// =================================================================
long long current_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

// =================================================================
// 1. KARTA, HJÄLPFUNKTIONER & RUTTPLANERING
// =================================================================
void init_karta() {
    memset(vag, 0, sizeof(vag));                             // Nollställ grannmatrisen
    memset(nodriktningsmatris, ' ', sizeof(nodriktningsmatris)); // Nollställ riktningsmatrisen

    for (int i = 0; i < 25; i++) {
        int rad = i / 5; // Vilken rad noden befinner sig på (0-4)
        int kol = i % 5; // Vilken kolumn noden befinner sig på (0-4)
        if (kol < 4) { vag[i][i+1] = 1; nodriktningsmatris[i][i+1] = 'e'; } // Kant österut
        if (kol > 0) { vag[i][i-1] = 1; nodriktningsmatris[i][i-1] = 'w'; } // Kant västerut
        if (rad < 4) { vag[i][i+5] = 1; nodriktningsmatris[i][i+5] = 's'; } // Kant söderut
        if (rad > 0) { vag[i][i-5] = 1; nodriktningsmatris[i][i-5] = 'n'; } // Kant norrut
    }

    // Koppla startnod (25) till nod 0 (övre vänstra hörnet)
    vag[START][0] = 1; 
    vag[0][START] = 1; 
    nodriktningsmatris[START][0] = 's'; // Från start kör vi söderut in på kartan
    nodriktningsmatris[0][START] = 'n'; // Från nod 0 kör vi norrut tillbaka till start
}

char get_turn(char nu, char nasta) {
    if (nu == nasta) return 'f'; // Rakt fram, ingen sväng
    // Högersvängar (medsols):
    if (nu == 'n' && nasta == 'e') return 'e'; 
    if (nu == 'e' && nasta == 's') return 'e';
    if (nu == 's' && nasta == 'w') return 'e';
    if (nu == 'w' && nasta == 'n') return 'e';
    // Vänstersvängar (motsols):
    if (nu == 'n' && nasta == 'w') return 'o'; 
    if (nu == 'w' && nasta == 's') return 'o';
    if (nu == 's' && nasta == 'e') return 'o';
    if (nu == 'e' && nasta == 'n') return 'o';
    return 'u'; // 180-gradersväng (U-sväng)
}

char get_motsatt_dir(char nu) {
    if (nu == 's') return 'n';
    if (nu == 'n') return 's';
    if (nu == 'e') return 'w';
    if (nu == 'w') return 'e';
    return nu; // Okänd riktning, returnera oförändrat
}

void bygg_beslut(int rutt[], char start_dir, char beslut[]) {
    char dir = start_dir; // Robotens nuvarande körriktning
    int i = 0; 

    while (rutt[i + 1] != STOP) {
        char nasta_dir = nodriktningsmatris[rutt[i]][rutt[i + 1]]; // Riktning till nästa nod
        beslut[i] = get_turn(dir, nasta_dir); // Beräkna sväng: f, e, o eller u
        dir = nasta_dir; // Uppdatera riktningen till vad vi körde
        i++;
    }
    beslut[i]   = 'X';  // 'X' = slutmarkering: "du är framme"
    beslut[i+1] = '\0'; // Null-terminera strängen
}

int hitta_rutt(int start, int mal, int rutt[], char start_dir) {
    int kostnad[NODES];       // Kortaste kända kostnad till varje nod
    int foregaende[NODES];    // Föregående nod på kortaste vägen
    char riktning_in[NODES];  // Inkommande riktning när vi nådde noden
    bool besokt[NODES] = {false}; // Dijkstras besökt-markering

    for (int i = 0; i < NODES; i++) { // Initiera alla noder som ej nådda
        kostnad[i]    = 9999;
        foregaende[i] = NONE;
        rutt[i]       = STOP;
    }

    kostnad[start]     = 0;         // Startnoden kostar 0
    riktning_in[start] = start_dir; // Startriktning sätts utifrån

    for (int i = 0; i < NODES; i++) { // Dijkstras huvudloop: NODES iterationer räcker
        int u = -1; // Välj noden med lägst kostnad som inte besökts
        for (int j = 0; j < NODES; j++) {
            if (!besokt[j] && (u == -1 || kostnad[j] < kostnad[u])) u = j; 
        }
        if (kostnad[u] == 9999 || u == mal) break; // Alla återstående är onåbara, eller målet nått
        besokt[u] = true; 

        for (int v = 0; v < NODES; v++) {
            if (vag[u][v] && !besokt[v]) { // För varje obesökt granne
                char nasta_dir = nodriktningsmatris[u][v]; 
                int straff     = (riktning_in[u] != nasta_dir) ? 1 : 0; // Svängstraff
                int ny_kostnad = kostnad[u] + 100 + straff; // 100 per kant + ev. svängstraff
                if (ny_kostnad < kostnad[v]) { // Relaxering: bättre väg hittad
                    kostnad[v]     = ny_kostnad;
                    foregaende[v]  = u;
                    riktning_in[v] = nasta_dir;
                }
            }
        }
    }

    // Bygg upp rutten bakifrån (från mål till start) och vänd den
    int temp[NODES], c = 0, nu = mal;
    while (nu != NONE) {
        temp[c++] = nu;
        nu = foregaende[nu];
    }

    for (int i = 0; i < c; i++) rutt[i] = temp[c - 1 - i];
    return kostnad[mal]; // Returnera total kostnad till målet
}

// Planerar rutt från from_node till varan (vara_u/vara_v).
// Fyller rutt_till_vara, beslut_till_vara.
// Sparar pickup_ingang, pickup_utgang, dir_vid_vara.
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

// Planerar hem från pickup-position till START.
// Fyller rutt_hem, beslut_hem (med 'f' eller 'u' prefix).
void planera_hem_fran_pickup() {
    int rutt_alt1[NODES], rutt_alt2[NODES];
    char dir_efter_vanding = get_motsatt_dir(dir_vid_vara);

    int cost_fwd = hitta_rutt(pickup_utgang, START, rutt_alt1, dir_vid_vara);
    int cost_utn = hitta_rutt(pickup_ingang, START, rutt_alt2, dir_efter_vanding) + 100;

    if (cost_fwd <= cost_utn) {
        memcpy(rutt_hem, rutt_alt1, sizeof(rutt_alt1));
        bygg_beslut(rutt_hem, dir_vid_vara, beslut_hem);
        int len = strlen(beslut_hem) + 1;
        memmove(&beslut_hem[1], &beslut_hem[0], len);
        beslut_hem[0] = 'f';
    } else {
        memcpy(rutt_hem, rutt_alt2, sizeof(rutt_alt2));
        bygg_beslut(rutt_hem, dir_efter_vanding, beslut_hem);
        int len = strlen(beslut_hem) + 1;
        memmove(&beslut_hem[1], &beslut_hem[0], len);
        beslut_hem[0] = 'u';
    }
}

// Planerar från pickup-position till nästa vara.
// Fyller beslut_till_vara (med 'f'/'u' prefix), uppdaterar pickup_ingang/utgang/dir.
void planera_nasta_vara() {
    int rutt_tmp[NODES];
    char dir_efter_vanding = get_motsatt_dir(dir_vid_vara);

    // Testa 4 kombinationer: (exit fwd/utn) × (approach vara_u/vara_v)
    int costs[4];
    costs[0] = hitta_rutt(pickup_utgang, vara_u, rutt_tmp, dir_vid_vara);
    costs[1] = hitta_rutt(pickup_utgang, vara_v, rutt_tmp, dir_vid_vara);
    costs[2] = hitta_rutt(pickup_ingang, vara_u, rutt_tmp, dir_efter_vanding) + 100;
    costs[3] = hitta_rutt(pickup_ingang, vara_v, rutt_tmp, dir_efter_vanding) + 100;

    int best = 0;
    for (int i = 1; i < 4; i++) { if (costs[i] < costs[best]) best = i; }

    bool uturn  = (best >= 2);
    int from_node = uturn ? pickup_ingang : pickup_utgang;
    char from_dir = uturn ? dir_efter_vanding : dir_vid_vara;
    int approach  = (best % 2 == 0) ? vara_u : vara_v;
    int through   = (approach == vara_u) ? vara_v : vara_u;

    // Planera rutt från exit-nod till approach-nod
    hitta_rutt(from_node, approach, rutt_till_vara, from_dir);
    int i = 0;
    while (rutt_till_vara[i] != STOP) i++;
    rutt_till_vara[i]   = through;
    rutt_till_vara[i+1] = STOP;

    bygg_beslut(rutt_till_vara, from_dir, beslut_till_vara);

    // Prefixera med 'f' eller 'u'
    int len = strlen(beslut_till_vara) + 1;
    memmove(&beslut_till_vara[1], &beslut_till_vara[0], len);
    beslut_till_vara[0] = uturn ? 'u' : 'f';

    // Uppdatera pickup-info för denna nya vara
    pickup_ingang = approach;
    pickup_utgang = through;
    dir_vid_vara  = nodriktningsmatris[approach][through];
}

// =================================================================
// 2. I2C, TELEMETRY & AUTO-INIT FUNKTIONER
// =================================================================

void log_verification(const unsigned char *sent, char action) {
    FILE *f = fopen(VERIFY_LOG_FILE, "a");
    if (f == NULL) return;
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    fprintf(f, "[%02d:%02d:%02d] ACTION '%c'\nSKICKAT (0x12): ", t->tm_hour, t->tm_min, t->tm_sec, action);
    for (int i = 0; i < PACKET_SIZE; i++) fprintf(f, "%02X ", sent[i]);
    fprintf(f, "\n\n");
    fclose(f);
}

void log_sensor_data(const unsigned char *received) {
    FILE *f = fopen(VERIFY_LOG_FILE, "a");
    if (f == NULL) return;
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    fprintf(f, "[%02d:%02d:%02d] SENSOR LÄST (0x10): ", t->tm_hour, t->tm_min, t->tm_sec);
    for (int i = 0; i < PACKET_SIZE; i++) fprintf(f, "%02X ", received[i]);
    fprintf(f, "\n\n");
    fclose(f);
}

void aktivt_beslut_fn(int index) {
    if (current_phase == PHASE_TO_ITEM) {
        aktivt_beslut = beslut_till_vara[index];
        nasta_beslut  = beslut_till_vara[index + 1]; // +1 för GUI-förhandsvisning
    } else if (current_phase == PHASE_PICKUP) {
        aktivt_beslut = 'v';
        nasta_beslut  = beslut_hem[0]; // Nästa fas börjar med hemruttens första beslut
    } else if (current_phase == PHASE_TO_HOME) {
        aktivt_beslut = beslut_hem[index];
        nasta_beslut  = beslut_hem[index + 1]; // +1 för GUI-förhandsvisning
    }
}

void start_autonomous_sequence(unsigned char state) {
    if (item_count <= 0) {
        printf("[!] No items configured. Send item list (0x07) first.\n");
        return;
    }

    current_item_index = 0;
    vara_u = item_list_u[0];
    vara_v = item_list_v[0];
    
    printf("\n=== AUTONOMOUS ROUTE: %d item(s) to collect ===\n", item_count);
    printf("-> Item 1/%d: edge %d <-> %d\n", item_count, vara_u, vara_v);
    planera_till_vara(START, 's');
    planera_hem_fran_pickup(); // Förberäkna hemrutt (kan ändras om fler varor finns)
    
    current_auto_state   = state;
    current_phase        = PHASE_TO_ITEM;
    current_action_index = 0;
    korsning_aktiv       = 0;
    loop_counter         = 0;

    aktivt_beslut_fn(current_action_index);
    current_node = rutt_till_vara[0];

    if (aktivt_beslut == 'e' || aktivt_beslut == 'o' || aktivt_beslut == 'u') {
        is_rotating = true;
        action_timer_start = current_time_ms();
    } else if (sim_sensor) {
        sim_segment_timer = current_time_ms();
    }

    log_next_action = true;
    printf("-> Route Calculated. Driving to item 1/%d...\n", item_count);
    if (sim_sensor) printf("[SIM] Intersections will be triggered every %d ms\n", SIM_SEGMENT_MS);
}

// =================================================================
// 3. HUVUDPROGRAM
// =================================================================
int main() {
    int sockfd, i2c_styr_fd, i2c_sens_fd; 
    struct sockaddr_in servaddr, cliaddr; 
    unsigned char buffer[BUFFER_SIZE]; 
    socklen_t len = sizeof(cliaddr); 

    uint8_t line_var = 0; // Linjesensorvärde från sensorkortet
    uint8_t angle    = 0; // Vinkeldata (används för debug/telemetri)
    uint8_t gyro1    = 0; // Gyroskopdata byte 1
    uint8_t gyro2    = 0; // Gyroskopdata byte 2
    
    uint8_t flags             = 0; // Råa flaggbitar från sensorkortet
    uint8_t flags_korsning    = 0; // Bits 2-3: 0=ingenstans, 1=pickup, 2=korsning
    uint8_t flags_ny_korsning = 0; // Bit 5: ny-korsning-puls (sticky tills vi rensar den)

    init_karta(); // Bygg upp grannmatris och riktningsmatris för 5x5-nätet

    // Rensa loggfilen vid uppstart
    FILE *clr = fopen(VERIFY_LOG_FILE, "w");
    if (clr) fclose(clr);
    printf("--- PI CORE: DUAL I2C (0x10 & 0x12) + UDP ROUTER ---\n");

    // Öppna I2C-anslutning till motorstyrningen (0x12)
    i2c_styr_fd = open(I2C_DEVICE, O_RDWR); 
    if (i2c_styr_fd >= 0) { 
        ioctl(i2c_styr_fd, I2C_SLAVE, STYRKOMM_ADDR); 
        if (write(i2c_styr_fd, NULL, 0) < 0) { 
            sim_motor = true;
            printf("[SIM] Motor Controller (0x12) missing. Motor writes disabled.\n"); 
        } else {
            printf("Connected to Motor Controller (0x12)\n"); 
        }
    } else {
        sim_motor = true;
        printf("[SIM] Could not open I2C for Motor Controller. Motor writes disabled.\n");
    }

    // Öppna I2C-anslutning till sensorkortet (0x10)
    i2c_sens_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_sens_fd >= 0) {
        ioctl(i2c_sens_fd, I2C_SLAVE, SENSOR_ADDR);
        if (write(i2c_sens_fd, NULL, 0) < 0) {
            sim_sensor = true;
            printf("[SIM] Sensor Board (0x10) missing. Using time-based intersection simulation (%d ms).\n", SIM_SEGMENT_MS);
        } else {
            printf("Connected to Sensor Board (0x10)\n");
        }
    } else {
        sim_sensor = true;
        printf("[SIM] Could not open I2C for Sensor Board. Using time-based intersection simulation (%d ms).\n", SIM_SEGMENT_MS);
    }

    if (sim_sensor || sim_motor) {
        printf("\n*** RUNNING IN SIM MODE ***\n\n");
    }

    // Skapa UDP-socket för kommunikation med GUI på datorn
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { 
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }
    
    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)); // Tillåt återanvändning av porten

    memset(&servaddr, 0, sizeof(servaddr)); 
    servaddr.sin_family      = AF_INET;    // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY; // Lyssna på alla nätverksgränssnitt
    servaddr.sin_port        = htons(UDP_PORT); 

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) { 
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }
    printf("Listening for UDP on port %d...\n\n", UDP_PORT);

    // NON-BLOCKING MAIN LOOP (~500 Hz via usleep(2000) i botten)
    while (1) {
        // -------------------------------------------------------------
        // 1. READ FROM SENSOR (0x10)
        // -------------------------------------------------------------
        unsigned char sensor_packet[PACKET_SIZE];
        if (!sim_sensor && i2c_sens_fd >= 0 && read(i2c_sens_fd, sensor_packet, PACKET_SIZE) == PACKET_SIZE) { 
            
            // Logga bara om paketet ändrats sedan sist (undviker logspam)
            static unsigned char last_sensor_packet[PACKET_SIZE] = {0};
            if (memcmp(sensor_packet, last_sensor_packet, PACKET_SIZE) != 0) {
                log_sensor_data(sensor_packet);
                memcpy(last_sensor_packet, sensor_packet, PACKET_SIZE);
            }
            
            flags    = sensor_packet[0]; // Byte 0: bitmaskade flaggor
            line_var = sensor_packet[1]; // Byte 1: linjesensorvärde
            angle    = sensor_packet[2]; // Byte 2: vinkel
            gyro1    = sensor_packet[6]; // Byte 6: gyro 1
            gyro2    = sensor_packet[7]; // Byte 7: gyro 2

            flags_korsning = (flags & 0x0C) >> 2; // Extrahera bits 2-3 för korsningstyp
            
            // Sticky-flagga för ny korsning: sätts till 1 när bit 5 är hög, rensas manuellt
            if (!flags_ny_korsning) {
                flags_ny_korsning = (flags & 0x20) >> 4; 
            }
        }

        // -------------------------------------------------------------
        // 2. CHECK FOR NETWORK PACKETS (INSTANTLY)
        // -------------------------------------------------------------
        int n = recvfrom(sockfd, buffer, BUFFER_SIZE, MSG_DONTWAIT, (struct sockaddr *)&cliaddr, &len);
        
        if (n > 0) {
            gui_known = true; // Vi vet nu GUI:ns IP och port, börja skicka telemetri
        }

        // Parsa inkomna paket: header 0x05, footer 0xFF, storlek PACKET_SIZE
        if (n == PACKET_SIZE && buffer[0] == 0x05 && buffer[7] == 0xFF) {
            unsigned char state  = buffer[1];
            unsigned char target = buffer[2];
            char action          = (char)buffer[3];

            if (state == 0x00 || state == 0x01) {
                if (action == 'f' && current_phase == PHASE_IDLE) {
                    start_autonomous_sequence(state);
                } else {
                    buffer[4] = line_var;
                    buffer[5] = gyro1;
                    buffer[6] = gyro2;
                    if (!sim_motor) write(i2c_styr_fd, buffer, PACKET_SIZE);
                    log_verification(buffer, action);
                    printf("-> Manual Command Forwarded: '%c'\n", action);
                }
            } else if (state == 0x02 || state == 0x03) {
                if (current_phase != PHASE_IDLE) {
                    printf("\n[!] MANUAL OVERRIDE DETECTED. Canceling Auto Route.\n");
                    current_phase        = PHASE_IDLE;
                    is_rotating          = false;
                    is_picking_up        = false;
                    current_action_index = 0;
                    korsning_aktiv       = 0;
                    aktivt_beslut        = 's';
                    nasta_beslut         = 's';
                }
                buffer[4] = line_var;
                buffer[5] = gyro1;
                buffer[6] = gyro2;
                if (!sim_motor) write(i2c_styr_fd, buffer, PACKET_SIZE);
                log_verification(buffer, action);
                printf("-> Manual Command Forwarded: '%c'\n", action);
            }
        }

        // Parsa item-list paket: header 0x07
        if (n >= 4 && buffer[0] == 0x07) {
            int num = buffer[1];
            int expected_len = 3 + 2 * num; // header + count + N*(u,v) + footer
            if (num > 0 && num <= MAX_ITEMS && n == expected_len && buffer[n-1] == 0xFF) {
                item_count = 0;
                for (int i = 0; i < num; i++) {
                    uint8_t iu = buffer[2 + 2*i];
                    uint8_t iv = buffer[3 + 2*i];
                    if (iu < NODES && iv < NODES && vag[iu][iv]) {
                        item_list_u[item_count] = iu;
                        item_list_v[item_count] = iv;
                        item_count++;
                    } else {
                        printf("[!] Skipping invalid item edge (%d, %d)\n", iu, iv);
                    }
                }
                current_item_index = 0;
                printf("[ITEMS] Received %d valid item(s) from GUI\n", item_count);
            } else {
                printf("[!] Invalid item-list packet (n=%d, count=%d)\n", n, num);
            }
        }

        // -------------------------------------------------------------
        // 3. AUTONOMOUS STATE MACHINE (INTERSECTION-BASED TRIGGERS)
        // -------------------------------------------------------------
        if (current_phase != PHASE_IDLE) { 
            
            long long elapsed_in_state = current_time_ms() - action_timer_start; // Millisekunder sedan start

            if (is_rotating) {
                // Tidsstyrd rotation: 500 ms inbromsning (hanteras automatiskt vid paket-sändningen)
                // Sedan 10000 ms sväng. Totalt 10500 ms innan vi kör framåt igen.
                if (elapsed_in_state >= 10500) { 
                    is_rotating   = false;
                    aktivt_beslut = 'f'; // Kör rakt efter avslutad rotation för att nå nästa nod
                    
                    if (current_phase == PHASE_TO_ITEM) {
                        nasta_beslut = beslut_till_vara[current_action_index + 1];
                    } else if (current_phase == PHASE_TO_HOME) {
                        nasta_beslut = beslut_hem[current_action_index + 1];
                    }
                    log_next_action = true;

                    // I sim-läge: starta segmenttimern för att simulera körning till nästa korsning
                    if (sim_sensor) sim_segment_timer = current_time_ms();
                }
            } 
            else if (is_picking_up) {
                // Pickup-sekvens i två steg (10000 millisekunder vardera):
                
                // Steg 1 (efter 10s): skicka pickup-kommando 'v' till mekaniken
                if (elapsed_in_state >= 10000 && aktivt_beslut == 's') {
                    aktivt_beslut = 'v';
                    // Förhandsvisning: nästa steg beror på om fler varor finns
                    if (current_item_index + 1 < item_count) {
                        nasta_beslut = 'f'; // Nästa vara
                    } else {
                        nasta_beslut = beslut_hem[0]; // Hem
                    }
                    log_next_action = true;
                }
                // Steg 2 (efter 20s): pickup klar, avgör nästa fas
                else if (elapsed_in_state >= 20000) {
                    is_picking_up = false;
                    current_item_index++;

                    if (current_item_index < item_count) {
                        // === FLER VAROR: planera rutt till nästa vara ===
                        vara_u = item_list_u[current_item_index];
                        vara_v = item_list_v[current_item_index];
                        printf("\n-> Item %d/%d: edge %d <-> %d\n",
                               current_item_index + 1, item_count, vara_u, vara_v);
                        planera_nasta_vara();
                        // Förberäkna hemrutt från nästa pickup (för GUI-förhandsvisning)
                        planera_hem_fran_pickup();

                        current_phase        = PHASE_TO_ITEM;
                        current_action_index = 0;
                        aktivt_beslut_fn(current_action_index);

                        if (aktivt_beslut == 'e' || aktivt_beslut == 'o' || aktivt_beslut == 'u') {
                            is_rotating = true;
                            action_timer_start = current_time_ms();
                        } else if (sim_sensor) {
                            sim_segment_timer = current_time_ms();
                        }
                        printf("-> PHASE CHANGE: Driving to item %d/%d...\n",
                               current_item_index + 1, item_count);
                        log_next_action = true;
                    } else {
                        // === SISTA VARAN PLOCKAD: kör hem ===
                        planera_hem_fran_pickup();
                        current_phase        = PHASE_TO_HOME;
                        current_action_index = 0;
                        aktivt_beslut_fn(current_action_index);

                        if (aktivt_beslut == 'e' || aktivt_beslut == 'o' || aktivt_beslut == 'u') {
                            is_rotating = true;
                            action_timer_start = current_time_ms();
                        } else if (sim_sensor) {
                            sim_segment_timer = current_time_ms();
                        }
                        printf("\n-> PHASE CHANGE: All %d items collected. Heading Home...\n", item_count);
                        log_next_action = true;
                    }
                }
            } 
            else {
                // --- KORSNINGSDETEKTION ---
                // I sim-läge: simulera korsning efter SIM_SEGMENT_MS ms
                // I riktigt läge: vänta på flaggan 'flags_korsning' från sensorn
                bool intersection_triggered = false;

                if (sim_sensor) {
                    // Tidbaserad simulering: en korsning "nås" efter SIM_SEGMENT_MS millisekunder
                    if (sim_segment_timer > 0 && (current_time_ms() - sim_segment_timer) >= SIM_SEGMENT_MS) {
                        intersection_triggered = true;
                        sim_segment_timer = 0; // Rensa timern, sätts igen vid nästa segment
                    }
                } else {
                    // Riktigt läge: använd sensorflaggor
                    if ((flags_korsning == 2 || flags_korsning == 1) && !korsning_aktiv) {
                        intersection_triggered = true;
                        korsning_aktiv    = 1;    // Debounce
                        flags_ny_korsning = 0;
                    } else if (flags_korsning == 0) {
                        korsning_aktiv = 0; // Återställ debounce
                    }
                }

                if (intersection_triggered) {
                    current_action_index++; // Gå vidare till nästa beslut i listan
                    aktivt_beslut_fn(current_action_index);
                    action_timer_start = current_time_ms(); // Starta timer för eventuell rotation

                    // Uppdatera aktuell nod baserat på rutten
                    if (current_phase == PHASE_TO_ITEM) {
                        current_node = rutt_till_vara[current_action_index];
                    } else if (current_phase == PHASE_TO_HOME) {
                        current_node = rutt_hem[current_action_index];
                    }

                    if (aktivt_beslut == 'e' || aktivt_beslut == 'o' || aktivt_beslut == 'u') {
                        // is_rotating tvingar automatiskt fram 500ms 's' i paket-byggaren nedan
                        is_rotating = true;
                        log_next_action = true;
                    }
                    else if (aktivt_beslut == 'X') {
                        if (current_phase == PHASE_TO_ITEM) {
                            current_phase = PHASE_PICKUP;
                            aktivt_beslut = 's';
                            if (current_item_index + 1 < item_count) {
                                nasta_beslut = 'f';
                            } else {
                                nasta_beslut = beslut_hem[0];
                            }
                            is_picking_up = true;
                            printf("\n-> Pickup item %d/%d...\n", current_item_index + 1, item_count);
                            log_next_action = true;
                        }
                        else if (current_phase == PHASE_TO_HOME) {
                            // Vi är hemma, avsluta autonom körning
                            current_phase = PHASE_IDLE;
                            current_node  = START; // Tillbaka till startnoden
                            aktivt_beslut = 's'; // Stoppa roboten
                            nasta_beslut  = 's'; // Rensa GUI-förhandsvisning
                            printf("\n=== AUTONOMOUS ROUTE COMPLETE ===\n\n");
                            
                            unsigned char stop_packet[PACKET_SIZE] = {
                                0x05, current_auto_state, 0x00, 's', 
                                line_var, gyro1, gyro2, 0xFF
                            };
                            if (!sim_motor) write(i2c_styr_fd, stop_packet, PACKET_SIZE);
                        }
                    }
                    else {
                        // Normalt beslut (f = rakt fram): kör vidare utan rotation
                        log_next_action = true;
                    }

                    // I sim-läge: starta segmenttimern för nästa korsning
                    // (om vi inte just startade en rotation — den har sin egen timer)
                    if (sim_sensor && !is_rotating && !is_picking_up && aktivt_beslut != 'X') {
                        sim_segment_timer = current_time_ms();
                    }
                }
            }

            // Skicka aktivt kommando till motorstyrningen varje loop-iteration
            if (current_phase != PHASE_IDLE && aktivt_beslut != 'X') {
                
                char skickat_kommando = aktivt_beslut;
                
                // --- MJUKT STOPP (500 ms) INNAN SVÄNG ---
                // Om vi ska svänga, åsidosätter vi beslutet och tvingar iväg 's' de första 500 millisekunderna.
                // Detta dödar robotens framåtmomentum innan den faktiskt börjar snurra.
                if (is_rotating && (current_time_ms() - action_timer_start < 500)) {
                    skickat_kommando = 's';
                }

                unsigned char auto_packet[PACKET_SIZE] = {
                    0x05,                                                                       // Header
                    current_auto_state,                                                         // Körläge
                    (current_phase == PHASE_PICKUP && aktivt_beslut == 'v') ? 0x01 : 0x00,      // Pickup-flagga
                    skickat_kommando,                                                           // Kommando (sväng, rakt fram eller mjukt stopp)
                    line_var, // Aktuell linjesensordata (skickas med för motorstyrningens PID)
                    gyro1,    // Gyrodata byte 1
                    gyro2,    // Gyrodata byte 2
                    0xFF      // Footer
                };

                if (!sim_motor) write(i2c_styr_fd, auto_packet, PACKET_SIZE);
                
                // Skriv ut till konsolen första gången ett nytt beslut sätts
                if (log_next_action) {
                    printf("Action updated to: '%c' (Sending to motors: '%c', Index: %d, Next: '%c')\n",
                           aktivt_beslut, auto_packet[3], current_action_index, nasta_beslut);
                    log_next_action = false;
                }

                // Logga till fil var 50:e iteration (~10 Hz) istället för varje loop (500 Hz)
                static int blasting_log_counter = 0;
                blasting_log_counter++;
                if (blasting_log_counter >= 50) { 
                    log_verification(auto_packet, auto_packet[3]);
                    blasting_log_counter = 0;
                }
            }
        }

        // -------------------------------------------------------------
        // 4. SKICKA TELEMETRI TILLBAKA TILL GUI (10Hz)
        // -------------------------------------------------------------
        if (gui_known) {
            telemetry_counter++;
            if (telemetry_counter >= 50) { // 50 iterationer * 2ms = 100ms = 10 Hz
                unsigned char telemetry_packet[(PACKET_SIZE+4)] = {
                    0x06,                         // 0x06 identifierar paketet som telemetri
                    (unsigned char)current_phase, // Aktuell fas i state machine
                    aktivt_beslut,                // Vad vi skickar till motorerna just nu
                    nasta_beslut,                 // Nästa beslut (index+1, förhandsvisning för GUI)
                    line_var,                     // Sensordata: Linje
                    gyro1,                        // Sensordata: Gyro 1
                    gyro2,                        // Sensordata: Gyro 2
                    flags,                        // Sensordata: Råa flaggor (bitmaskade)
                    current_node,                 // Robotens aktuella nod (0-25)
                    (unsigned char)current_item_index, // Vilken vara vi är på (0-baserat)
                    (unsigned char)item_count,         // Totalt antal varor
                    0xFF                          // Footer
                };
                sendto(sockfd, telemetry_packet, (PACKET_SIZE+4), 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
                telemetry_counter = 0;
            }
        }
        
        // -------------------------------------------------------------
        // 5. TINY DELAY (2000 microseconds = 2 milliseconds / 500Hz)
        // -------------------------------------------------------------
        usleep(2000); 
    }

    close(sockfd);
    if (i2c_styr_fd >= 0) close(i2c_styr_fd);
    if (i2c_sens_fd >= 0) close(i2c_sens_fd);
    return 0;
}