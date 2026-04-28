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
#include <stdint.h> // Required for int16_t and uint8_t

// --- ALGORITHM DEFINITIONS ---
#define NODES 26 // 5x5 samt en start/slutnod
#define START 25 // Start/slut på nod 25
#define NONE -1 // Betyder att det inte finns en föregående nod
#define STOP -1 // Stoppvillkor för ruttarray

// --- TELEMETRY DEFINITIONS ---
#define UDP_PORT 5001 // Porten som används för kommunikation med persondatorn
#define BUFFER_SIZE 1024 // Storleken på bufferten
#define I2C_DEVICE "/dev/i2c-1" // Filnamn för i2c-bussen
#define STYRKOMM_ADDR 0x12
#define SENSOR_ADDR 0x10
#define PACKET_SIZE 8 // Paketstorleken för kommunikationen inom systemet (i2c)
#define VERIFY_LOG_FILE "verifikation_keys.txt" // Loggfil

// --- ALGORITHM GLOBALS ---
char nodriktningsmatris[NODES][NODES]; // Väderstreck för vägarna (ex från (5) till (6) är östlig riktning 'e')
int  vag[NODES][NODES]; // Huruvida vägen är aktiv/finns (ex väg mellan (0) och (20) finns ej och är 0)
int  rutt_till_vara[NODES]; // Array med vägen (nodnummer) till varan
int  rutt_hem[NODES]; // Array med vägen (nodnummer) från varan till slut
char beslut_till_vara[NODES]; // Array med tecken (action) vid varje korsning
char beslut_hem[NODES]; // Array med tecken (ACTION) vid varje korning hem
int  vara_u, vara_v; // u och v är noder och varan ligger mellan dem och bestäms av input från persondatorn

// --- STATE MACHINE GLOBALS ---
// Definierar datatypen 'autophase' som bara kan ta fyra värden
// Används till att bestämma vad roboten håller på med just nu
typedef enum {
    PHASE_IDLE = 0,
    PHASE_TO_ITEM,
    PHASE_PICKUP,
    PHASE_TO_HOME
} AutoPhase;

AutoPhase current_phase = PHASE_IDLE; // I början står roboten still
int current_action_index = 0; // Index för att hämta aktuellt belut. Räknas upp när vi når korsning.
unsigned char current_auto_state = 1; // Bestämmer STATE (auto, auto), (manuell, auto) osv
bool log_next_action = false; // Har med logg att göra

// Nya variabler för den icke-blockerande "blasting"-logiken
bool is_rotating = false;
bool is_picking_up = false;
time_t action_timer_start = 0; // Timer för att ersätta styrmodulens svarsignal

// --- TELEMETRY GLOBALS FÖR GUI ---
bool gui_known = false; // Har GUI:t hört av sig än?
int telemetry_counter = 0; // Räknare för att inte spamma nätverket

// --- LOOP TIMING GLOBALS ---
char nasta_beslut  = 's'; // Nästa beslut från beslutsarrayen (e, o, u, f, X osv)
char aktivt_beslut = 's'; // Det som faktiskt skickas till styrmodulen ('f' nästan hela tiden, annars rotation/stopp)
int  loop_counter = 0; // Räknar upp loopen för att se när nästa beslut ska skickas (just nu bara test)

// =================================================================
// 1. KARTA, HJÄLPFUNKTIONER & RUTTPLANERING
// =================================================================
void init_karta() {
    memset(vag, 0, sizeof(vag)); // Sätter alla vägar inaktiva
    memset(nodriktningsmatris, ' ', sizeof(nodriktningsmatris)); // Sätter alla väderstreck till blankkaraktärer

    // Loopar igenom alla noder
    for (int i = 0; i < 25; i++) {
        int rad = i / 5; // Räknar ut vilken rad noden är på genom att avrunda aktuell nod / antal kolonner nedåt
        int kol = i % 5; // Räkanr ut vilken kolonn noden är på genom att ta resten av aktuell nod / antal rader 
        if (kol < 4) { vag[i][i+1] = 1; nodriktningsmatris[i][i+1] = 'e'; } // Noder på kolonn 0-3 har alltid en nod österut 'e' som är nästa nodnummer
        if (kol > 0) { vag[i][i-1] = 1; nodriktningsmatris[i][i-1] = 'w'; } // Noder på kolonn 1-4 har alltid en nod västerut 'w' som är föregående nodnummer
        if (rad < 4) { vag[i][i+5] = 1; nodriktningsmatris[i][i+5] = 's'; } // Noder på rad 0-3 har alltid en nod söderut 's' som är 5 nodnummer framåt
        if (rad > 0) { vag[i][i-5] = 1; nodriktningsmatris[i][i-5] = 'n'; } // Noder på rad 1-4 har alltid en nod norrut 'n' som är 5 nodnummer bakåt
    }

    vag[START][0] = 1; // Aktivera vägen mellan startnod och nod 0 
    vag[0][START] = 1; // Aktivera vägen mellan nod 0 och startnod (symmetrisk matris)
    nodriktningsmatris[START][0] = 's'; // Förutsätter att start -> 0 är söderut
    nodriktningsmatris[0][START] = 'n'; // Förutsäter att 0 -> start är norrut
}

// Funktion som avgör svängriktning genom att jämföra väderstreck
char get_turn(char nu, char nasta) {
    if (nu == nasta) return 'f';
    if (nu == 'n' && nasta == 'e') return 'e'; // Rotation medsols
    if (nu == 'e' && nasta == 's') return 'e';
    if (nu == 's' && nasta == 'w') return 'e';
    if (nu == 'w' && nasta == 'n') return 'e';
    if (nu == 'n' && nasta == 'w') return 'o'; // Rotation motsols
    if (nu == 'w' && nasta == 's') return 'o';
    if (nu == 's' && nasta == 'e') return 'o';
    if (nu == 'e' && nasta == 'n') return 'o';
    return 'u'; // Rotation 180 grader
}

// Funktion som anger motsatt riktning
char get_motsatt_dir(char nu) {
    if (nu == 's') return 'n';
    if (nu == 'n') return 's';
    if (nu == 'e') return 'w';
    if (nu == 'w') return 'e';
    return nu; 
}

// Funktion som tar in en rutt och robotens startriktning och uppdaterar beslutsarrayen
void bygg_beslut(int rutt[], char start_dir, char beslut[]) {
    char dir = start_dir; // Hjälpvariabel
    int i = 0; // Index

    // Loopa till dess att det inte finns något mer i rutten
    while (rutt[i + 1] != STOP) {
        char nasta_dir = nodriktningsmatris[rutt[i]][rutt[i + 1]]; // Kollar vilken väderstreck noden har jämfört med nästa nod i rutten
        beslut[i] = get_turn(dir, nasta_dir); // Jämför väderstreck med nästa väderstreck för att beräkna riktning och lagra i beslutet
        dir = nasta_dir;
        i++;
    }
    beslut[i] = 'X'; // Sista beslutet är 'X' som betyder hämta/lämna varan
    beslut[i+1] = '\0'; // Avslutar array
}

// Funktion som beräknar rutten genom BFS + kostnad för svängar
int hitta_rutt(int start, int mal, int rutt[], char start_dir) {
    int kostnad[NODES]; // Varje nod har en kostnad för att ta sig dit
    int foregaende[NODES]; // Varje nod har en föregående för att hålla koll på snabbaste vägen
    char riktning_in[NODES]; // n, s, e, w
    bool besokt[NODES] = {false}; // Huruvida noden är besökt eller ej

    for (int i = 0; i < NODES; i++) { // Sätt alla nodkostnader i början till oo
        kostnad[i] = 9999;
        foregaende[i] = NONE;
        rutt[i] = STOP;
    }

    kostnad[start] = 0; // Ingen kostnad för startnoden
    riktning_in[start] = start_dir; // Väderstreck in till startnod (robotens aktuella position)

    for (int i = 0; i < NODES; i++) { // Antal iterationer är mindre än antal noder
        int u = -1; // u är aktuell nod som besöks i iterationen
        for (int j = 0; j < NODES; j++) {
            if (!besokt[j] && (u == -1 || kostnad[j] < kostnad[u])) u = j; // Hittar billigaste ej besökta nod och tilldelar u
        }
        if (kostnad[u] == 9999 || u == mal) break; // Antingen om ingen väg finns eller målnod funnen, breaka
        besokt[u] = true; // Annars börjar vi besöka nod u

        // Försöker hitta närliggande obesökta noder
        for (int v = 0; v < NODES; v++) {
            if (vag[u][v] && !besokt[v]) { // Om vägen finns och ej är besökt...
                char nasta_dir = nodriktningsmatris[u][v]; // 1. Hitta väderstreck till noden v
                int straff = (riktning_in[u] != nasta_dir) ? 1 : 0; // 2. Straffa med 1 om väderstrecken mellan u och v skiljer sig (pga rotation)
                int ny_kostnad = kostnad[u] + 100 + straff; // 3. Beräkna kostnaden för v  
                if (ny_kostnad < kostnad[v]) { // Om ny kostnad < befintlig kostnad för nod v uppdateras v, föregående för v samt billigaste riktningen in i v
                    kostnad[v] = ny_kostnad;
                    foregaende[v] = u;
                    riktning_in[v] = nasta_dir;
                }
            }
        }
    }

    // Funktion som beräknar rutten baklänges 
    int temp[NODES], c = 0, nu = mal;
    while (nu != NONE) {
        temp[c++] = nu;
        nu = foregaende[nu];
    }

    // Hjälpfunktion för att sortera om rutten så att den blir i rätt ordning
    for (int i = 0; i < c; i++) rutt[i] = temp[c - 1 - i];
    return kostnad[mal];
}

// Beräknar beslut till vara och beslut hem oavsett var roboten befinner sig i kartan
void planera_hela_resan(int nuvarande_nod, char nuvarande_dir) { // Tar in vilken nod vi är på och i vilket väderstreck roboten är vänd mot
    int rutt_alt1[NODES], rutt_alt2[NODES]; // Temporära variabler för rutt

    // Vilken av de närliggande noderna till varan ska vi köra in genom för att det ska bli billigast? 
    int kostnad1 = hitta_rutt(nuvarande_nod, vara_u, rutt_alt1, nuvarande_dir);
    int kostnad2 = hitta_rutt(nuvarande_nod, vara_v, rutt_alt2, nuvarande_dir);
    
    int ingang, utgang;
    if (kostnad1 <= kostnad2) {
        ingang = vara_u; utgang = vara_v; // Om kostnad1 <= kostnad2 blir ingången via u och utgången via v
        memcpy(rutt_till_vara, rutt_alt1, sizeof(rutt_alt1)); // rutt_till_vara tilldelas rutt_alt1 med storlek rutt_alt1
    } else {
        ingang = vara_v; utgang = vara_u; // Tvärtom
        memcpy(rutt_till_vara, rutt_alt2, sizeof(rutt_alt2));
    }

    // Roboten måste åka mellan målnoderna och därför sätter vi utgångsnoden till sista noden, men ej säkert att den åker genom/till den
    int i = 0;
    while (rutt_till_vara[i] != STOP) i++;
    rutt_till_vara[i] = utgang;
    rutt_till_vara[i+1] = STOP;

    bygg_beslut(rutt_till_vara, nuvarande_dir, beslut_till_vara); // Omvandla till beslut
    
    char dir_vid_vara = nodriktningsmatris[ingang][utgang]; // Beräknar vilket väderstreck roboten har när den hämtat varan 
    int cost_utgang = hitta_rutt(utgang, START, rutt_alt1, dir_vid_vara);
    int cost_ingang = hitta_rutt(ingang, START, rutt_alt2, dir_vid_vara) + 100; 

    // Bestämmer huruvida roboten ska köra vidare till nästa nod eller vända och köra tillbaka (straff 100) efter att ha hämtat varan
    if (cost_utgang <= cost_ingang) {
        memcpy(rutt_hem, rutt_alt1, sizeof(rutt_alt1));
        bygg_beslut(rutt_hem, dir_vid_vara, beslut_hem);
    } else {
        memcpy(rutt_hem, rutt_alt2, sizeof(rutt_alt2));
        char dir_efter_vanding = get_motsatt_dir(dir_vid_vara);
        bygg_beslut(rutt_hem, dir_efter_vanding, beslut_hem);
        int len = strlen(beslut_hem) + 1;
        memmove(&beslut_hem[1], &beslut_hem[0], len);
        beslut_hem[0] = 'u'; 
    }
}

// =================================================================
// 2. I2C, TELEMETRY & AUTO-INIT FUNKTIONER
// =================================================================

// Initierar verifikationsfilen för utgående data
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

// Loggar sensordata i samma verifikationsfil
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

// Läser nästa beslut ur beslutsarrayen och lagrar i nasta_beslut (påverkar EJ aktivt_beslut direkt)
void aktivt_beslut_fn(int index) {
    if (current_phase == PHASE_TO_ITEM) {
        nasta_beslut = beslut_till_vara[index];
    } else if (current_phase == PHASE_PICKUP) {
        nasta_beslut = 'v'; // Arm pickup action OBS behöver uppdateras med faktiska beslut
    } else if (current_phase == PHASE_TO_HOME) {
        nasta_beslut = beslut_hem[index];
    }
}

// Här sätts var varan ligger någonstans, behöver uppdateras och integreras med guin
void start_autonomous_sequence(unsigned char state) {
    vara_u = 10; 
    vara_v = 11; 
    
    printf("\n=== CALCULATING AUTONOMOUS ROUTE ===\n");
    planera_hela_resan(START, 's'); // Vi är pån startnoden och roboten är PRELIMINÄRT riktad söderut
    
    current_auto_state = state; // Kan vara (auto, auto) eller (auto, manuell)
    current_phase = PHASE_TO_ITEM; // Kör mot varan
    current_action_index = 0; // Var i beslutslistan vi är
    
    loop_counter = 0; // Endast vid simulering
    aktivt_beslut_fn(current_action_index); // Läs första beslutet ur arrayen
    
    if (nasta_beslut == 'e' || nasta_beslut == 'o' || nasta_beslut == 'u') {
        aktivt_beslut = nasta_beslut; // Starta direkt med rotation
        is_rotating = true;
        action_timer_start = time(NULL); // Starta klockan för sekvensen
    } else {
        aktivt_beslut = 'f';
    }

    log_next_action = true; // Har med logg att göra
    
    printf("-> Route Calculated. Driving to item...\n");
}

// =================================================================
// 3. HUVUDPROGRAM
// =================================================================
int main() {
    int sockfd, i2c_styr_fd, i2c_sens_fd; // Deklarera filbeskrivningsnummer, används till R/WR
    struct sockaddr_in servaddr, cliaddr; // Deklarera familj (IPV4 eller IPV6), portnummer och IP-adress
    unsigned char buffer[BUFFER_SIZE]; // Deklarera storlek på buffer
    socklen_t len = sizeof(cliaddr); 

    // Placeholders till sensorvärdena
    uint8_t line_var = 0;
    uint8_t angle = 0;
    uint8_t gyro1 = 0;
    uint8_t gyro2 = 0;
    
    uint8_t flags = 0;
    uint8_t flags_korsning = 0;
    uint8_t flags_ny_korsning = 0;

    init_karta(); // Initierar kartan

    FILE *clr = fopen(VERIFY_LOG_FILE, "w");
    if (clr) fclose(clr);
    printf("--- PI CORE: DUAL I2C (0x10 & 0x12) + UDP ROUTER ---\n");

    // SETUP I2C - STYRKOMM (0x12)
    i2c_styr_fd = open(I2C_DEVICE, O_RDWR); // Öppnar fil till styrmodulen som ger filbeskrivningsnummer som kan läsas och skrivas till
    if (i2c_styr_fd >= 0) { // Om filen är gick att öppna...
        ioctl(i2c_styr_fd, I2C_SLAVE, STYRKOMM_ADDR); // Tilldela egenskaper till filen (slav, adress)
        if (write(i2c_styr_fd, NULL, 0) < 0) { // Om det inte går att fysiskt skriva till adressen...
            printf("[WARNING] Motor Controller (0x12) missing. Running in Sim Mode.\n"); // kör i simulerat läge
        } else {
            printf("Connected to Motor Controller (0x12)\n"); // Annars, kör som tänkt
        }
    }

    // SETUP I2C - SENSOR (0x10)
    i2c_sens_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_sens_fd >= 0) {
        ioctl(i2c_sens_fd, I2C_SLAVE, SENSOR_ADDR);
        if (write(i2c_sens_fd, NULL, 0) < 0) {
            printf("[WARNING] Sensor Board (0x10) missing. Will send zeros.\n");
        } else {
            printf("Connected to Sensor Board (0x10)\n");
        }
    }

    // SETUP UDP
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { // Skapar socket för att kunna kommunicera med persondatorn
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }
    
    // TILLÄGG: Tillåt återanvändning av adress/port direkt så vi slipper "Bind failed"-felet
    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&servaddr, 0, sizeof(servaddr)); // Ladda pekaren med nollor
    servaddr.sin_family = AF_INET; // IPV4 eller IPV6
    servaddr.sin_addr.s_addr = INADDR_ANY; // Tar in IP-adressen för persondatorn
    servaddr.sin_port = htons(UDP_PORT); // Sätter porten att kommunicera via

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) { // Binder socket till IP-adressen
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }
    printf("Listening for UDP on port %d...\n\n", UDP_PORT);

    // NON-BLOCKING MAIN LOOP
    while (1) {
        // -------------------------------------------------------------
        // 1. READ FROM SENSOR (0x10)
        // -------------------------------------------------------------
        unsigned char sensor_packet[PACKET_SIZE];
        if (i2c_sens_fd >= 0 && read(i2c_sens_fd, sensor_packet, PACKET_SIZE) == PACKET_SIZE) { // Om filen är öppen och PACKET_SIZE == 8... (read läser även in sensorpaketet)
           
            // Logga endast när datan från sensorn faktiskt ändras (för att inte spamma sönder filen)
            static unsigned char last_sensor_packet[PACKET_SIZE] = {0};
            if (memcmp(sensor_packet, last_sensor_packet, PACKET_SIZE) != 0) {
                log_sensor_data(sensor_packet);
                memcpy(last_sensor_packet, sensor_packet, PACKET_SIZE);
            }
            
            flags = sensor_packet[0];
            line_var = sensor_packet[1];
            angle = sensor_packet[2];
            gyro1 = sensor_packet[6];
            gyro2 = sensor_packet[7];

            flags_korsning = (flags & 0x0C) >> 2; // bit 2-3 extraherade -> 0=ingen, 1=pickup, 2=korsning
            // flags_ny_korsning sätts BARA om vi inte redan har en okonsumerad flagga,
            // annars skriver sensorn över den innan state machine hunnit agera
            if (!flags_ny_korsning) {
                flags_ny_korsning = (flags & 0x20) >> 4; // bit 5: ny feature detekterad
            }
        }

        // -------------------------------------------------------------
        // 2. CHECK FOR NETWORK PACKETS (INSTANTLY)
        // -------------------------------------------------------------

        // Läs in från persondatorn
        int n = recvfrom(sockfd, buffer, BUFFER_SIZE, MSG_DONTWAIT, (struct sockaddr *)&cliaddr, &len);
        
        if (n > 0) {
            gui_known = true; // Vi har fått ett paket, nu vet vi GUI:ts IP och port!
        }

        if (n == PACKET_SIZE && buffer[0] == 0x05 && buffer[7] == 0xFF) {
            unsigned char state = buffer[1];
            unsigned char target = buffer[2];
            char action = (char)buffer[3];

            if (state == 0x00 || state == 0x01) {
                // Start Auto Mode if 'f' is pressed
                if (action == 'f' && current_phase == PHASE_IDLE) {
                    start_autonomous_sequence(state);
                } else if (current_phase != PHASE_IDLE) {
                    // Inject Sensor Data before forwarding!
                    buffer[4] = line_var;
                    buffer[5] = gyro1;
                    buffer[6] = gyro2;

                    write(i2c_styr_fd, buffer, PACKET_SIZE);
                    log_verification(buffer, action);
                    printf("-> Manual Command Forwarded: '%c'\n", action);
                }
            } else if (state == 0x02 || state == 0x03) {
                // MANUAL OVERRIDE
                if (current_phase != PHASE_IDLE) {
                    printf("\n[!] MANUAL OVERRIDE DETECTED. Canceling Auto Route.\n");
                    current_phase = PHASE_IDLE;
                    is_rotating = false;
                    is_picking_up = false;
                }
                
                // Inject Sensor Data before forwarding!
                buffer[4] = line_var;
                buffer[5] = gyro1;
                buffer[6] = gyro2;

                write(i2c_styr_fd, buffer, PACKET_SIZE);
                log_verification(buffer, action);
                printf("-> Manual Command Forwarded: '%c'\n", action);
            }
        }

        // -------------------------------------------------------------
        // 3. AUTONOMOUS STATE MACHINE (INTERSECTION-BASED TRIGGERS)
        // -------------------------------------------------------------
        if (current_phase != PHASE_IDLE) { // Simulator för autonomt läge
            
            time_t elapsed_in_state = time(NULL) - action_timer_start;

            if (is_rotating) {
                // Efter 3 sekunders rotation (justerbart) anser vi svängen klar och kör framåt igen
                if (elapsed_in_state >= 3) { 
                    is_rotating = false;
                    aktivt_beslut = 'f';
                    log_next_action = true;
                }
            } 
            else if (is_picking_up) {
                // Efter 2 sekunder stoppat vid varan byter vi till arm-aktion 'v'
                if (elapsed_in_state >= 2 && aktivt_beslut == 's') {
                    aktivt_beslut = 'v';
                    log_next_action = true;
                }
                // Efter totalt 5 sekunder (2s stopp + 3s plock) är fasen klar
                else if (elapsed_in_state >= 5) {
                    is_picking_up = false;
                    current_phase = PHASE_TO_HOME;
                    current_action_index = 0; // Index nollställs vid fasövergång
                    aktivt_beslut_fn(current_action_index); // Läs första beslutet för hemresan
                    
                    if (nasta_beslut == 'e' || nasta_beslut == 'o' || nasta_beslut == 'u') {
                        aktivt_beslut = nasta_beslut; // Sätt rotation som aktivt beslut
                        is_rotating = true;
                        action_timer_start = time(NULL);
                    } else {
                        aktivt_beslut = 'f';
                    }
                    printf("\n-> PHASE CHANGE: Heading Home...\n");
                    log_next_action = true;
                }
            } 
            else {
                // NORMAL KÖRNING: Vi väntar på flaggan 'flags_korsning' från sensorn
                // flags_korsning == 2 betyder FEATURE_INTERSECTION, pickup (1) ska ej räkna upp index
                
                static uint8_t korsning_aktiv = 0; // Håller koll på om vi redan befinner oss på en korsning
                
                if (flags_korsning == 2 && !korsning_aktiv) {
                    korsning_aktiv = 1;    // Vi är på korsningen, sätt flaggan så vi inte triggar igen
                    flags_ny_korsning = 0; // Konsumera flaggan så att den inte triggar igen nästa loopvarv
                    
                    // Index uppdateras BARA HÄR – en gång per korsning
                    current_action_index++;
                    aktivt_beslut_fn(current_action_index); // Läs nästa beslut ur arrayen
                    action_timer_start = time(NULL); // Nollställ timern för den nya manövern

                    if (nasta_beslut == 'e' || nasta_beslut == 'o' || nasta_beslut == 'u') {
                        // Skicka stopp först (säkerställer renare rotation)
                        unsigned char stop_packet[PACKET_SIZE] = {
                            0x05, current_auto_state, 0x00, 's', 
                            line_var, gyro1, gyro2, 0xFF
                        };
                        write(i2c_styr_fd, stop_packet, PACKET_SIZE);
                        
                        aktivt_beslut = nasta_beslut; // Sätt rotation som aktivt beslut
                        is_rotating = true;
                        log_next_action = true;
                    }
                    else if (nasta_beslut == 'X') {
                        if (current_phase == PHASE_TO_ITEM) {
                            // Vi är framme vid varan, stanna först
                            aktivt_beslut = 's';
                            is_picking_up = true;
                            printf("\n-> PHASE CHANGE: Stopping before pickup...\n");
                            log_next_action = true;
                        }
                        else if (current_phase == PHASE_TO_HOME) {
                            // Vi är tillbaka vid startnoden
                            current_phase = PHASE_IDLE;
                            aktivt_beslut = 's';
                            printf("\n=== AUTONOMOUS ROUTE COMPLETE ===\n\n");
                            
                            unsigned char stop_packet[PACKET_SIZE] = {
                                0x05, current_auto_state, 0x00, 's', 
                                line_var, gyro1, gyro2, 0xFF
                            };
                            write(i2c_styr_fd, stop_packet, PACKET_SIZE);
                        }
                    }
                    else {
                        // Beslutet är 'f', fortsätt bara rakt framåt
                        aktivt_beslut = 'f';
                        log_next_action = true;
                    }
                } 
                else if (flags_korsning != 2) {
                    korsning_aktiv = 0; // Vi har rullat av korsningen, redo för nästa!
                }
            }

            // BLAST THE CURRENT ACTION CONTINUOUSLY (Varje varv i loopen, 500Hz)
            if (current_phase != PHASE_IDLE && aktivt_beslut != 'X') {
                
                unsigned char auto_packet[PACKET_SIZE] = {
                    0x05, 
                    current_auto_state, 
                    (current_phase == PHASE_PICKUP && aktivt_beslut == 'v') ? 0x01 : 0x00, // Arm or Wheel
                    aktivt_beslut, 
                    line_var,  // Inject calculated line sensor
                    gyro1,     // Inject direct gyro 1
                    gyro2,     // Inject direct gyro 2
                    0xFF
                };

                // Skicka till mikrokontrollern (I2C) 500ggr/s
                write(i2c_styr_fd, auto_packet, PACKET_SIZE);
                
                // Terminalutskrift vid förändring
                if (log_next_action) {
                    printf("Action updated to: '%c' (Index: %d)\n", auto_packet[3], current_action_index);
                    log_next_action = false;
                }

                // Loggning till fil ( throttlad till 10Hz för att inte skada SD-kortet)
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
            if (telemetry_counter >= 50) { 
                unsigned char telemetry_packet[PACKET_SIZE] = {
                    0x06,                         // 0x06 identifierar paketet som telemetri
                    (unsigned char)current_phase, // Aktuell fas
                    aktivt_beslut,                // Vad vi skickar till motorerna
                    line_var,                     // Sensordata: Linje
                    gyro1,                        // Sensordata: Gyro 1
                    gyro2,                        // Sensordata: Gyro 2
                    flags,                        // Sensordata: Flaggor (bitmaskade)
                    0xFF                          // Footer
                };
                sendto(sockfd, telemetry_packet, PACKET_SIZE, 0, (struct sockaddr *)&cliaddr, len);
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