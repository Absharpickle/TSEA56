//-------------------------------------------------------
// Markus Hellers, Joel Eberhardsson - 29 Maj 2026 - V1.0
//-------------------------------------------------------
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
#include <stdint.h>
#include <time.h>

//LÄS HEADERFILERNA!
#include "pathfinding.h"
#include "protocol.h"

#define SIM_SEGMENT_MS 1000 //för simulation
#define SLEEP 25000 //väntetid för raspberry pi så den inte ber om informaiton för ofta


typedef enum {  //status maskin som säger vilken fas roboten är i
    PHASE_IDLE = 0,  //vilande
    PHASE_TO_ITEM,   //påväg mot vara
    PHASE_PICKUP,    //plockar upp vara
    PHASE_TO_HOME,   //åker hem
    PHASE_DROP       //lämnar vara
} AutoPhase;         //namn på datatypen

AutoPhase current_phase      = PHASE_IDLE; //börjar stillastående
int current_action_index     = 0;          //index som används i beslutsarrayen
unsigned char current_auto_state = 1;      //status som avgör manuell/automatisk för arm och hjul 
bool log_next_action         = false;      //används för att logga sensor och styrinformation till ett textdoument för felsökning

bool is_rotating   = false;                 //boolean för om roboten roterar just nu
char pending_rotation_cmd = ' ';            //används för att spara rotationsriktning innan en rotation
bool is_picking_up = false;                 //om roboten plockar just nu
char pickup_cmd    = 'v';                   //upphämtningskommando beroende på varans position höger/vänster
bool is_dropping   = false;                 //om roboten lämnar vara just nu
bool is_hinder = false;                     //om det finns ett hinder framför
bool is_hinder2 = false;                    //används för att låsa hinderhanteringen
bool drop_step_done = false;                //används för att lämna varan    
long long action_timer_start = 0;           //tid när nuvarande åtgärd startade
uint8_t korsning_aktiv = 0;                 //om man står i en korsning (undvika dubbel triggande)

bool sim_sensor = false;            //simulera sensor om den inte finns
bool sim_motor  = false;            //simulera styrmodul om den inte finns
long long sim_segment_timer = 0;    //tid för simulerad körsträcka

bool gui_known        = false;      
int telemetry_counter = 0;          
bool route_changed    = false; 

char nasta_beslut  = 's';           //nästa beslut, i början "stopp"
char aktivt_beslut = 's';           //beslutet som körs just nu
int  loop_counter  = 0;             //räknare
uint8_t current_node = START;       //noden roboten befinner sig i 
char current_dir = 's';             //riktningen roboten är vänd mot
uint8_t action_done = 0;            //används när styrmodulen skickar klartecken
uint8_t styr_gas_right = 0;         //gaspådrag höger från styr
uint8_t styr_gas_left  = 0;         //gaspådrag vänster från vänster
int8_t  styr_claw_r    = 0;         //klo-position (rotation) från styr
int8_t  styr_claw_z    = 0;         //klo-position (höjd) från styr 
bool rotation_done = false;         //om rotation är färdig
bool pickup_step_done = false;      //om hämtande av vara är färdig

int flag_timer = 0;                 //räknare för korsningsflagga
int temp_flag = 0;                  //tidigare värde på korsningsflagga
int hinder_counter = 0;             //räknare för antal hinder
int hinder_timer = 0;               //timer för hinderupptäckning

int sockfd = -1;            //filbeskrivning ej öppen
int i2c_styr_fd = -1;       //filbeskrivning ej öppen
int i2c_sens_fd = -1;       //filbeskrivning ej öppen
struct sockaddr_in cliaddr;     //adress til gui

uint8_t line_var_f = 0;     //linjeföljning främre
uint8_t line_var_b = 0;     //linjeföljning bakre
uint8_t angle      = 0;     //vinkel
uint8_t gyro1      = 0;     //gyro 
uint8_t gyro2      = 0;     //gyro

//statusflaggor som tas från sensormodulen
uint8_t flags             = 0;      
uint8_t flags_korsning    = 0;
uint8_t flags_ny_korsning = 0;
uint8_t flags_ir          = 0;

//resetfunktion
void reset() {
    current_phase        = PHASE_IDLE;
    current_action_index = 0;
    current_auto_state   = 1;
    log_next_action      = false;

    is_rotating          = false;
    pending_rotation_cmd = ' ';
    is_picking_up        = false;
    pickup_cmd           = 'v';
    is_dropping          = false;
    is_hinder            = false;
    is_hinder2           = false;
    drop_step_done       = false;
    action_timer_start   = 0;
    korsning_aktiv       = 0;

    sim_sensor           = false;
    sim_motor            = false;
    sim_segment_timer    = 0;

    // gui_known kanske du vill behålla som true om GUI:t redan är anslutet?
    telemetry_counter    = 0;
    route_changed        = false;

    nasta_beslut         = 's';
    aktivt_beslut        = 's';
    loop_counter         = 0;
    current_node         = START;
    current_dir          = 's';
    action_done          = 0;
    rotation_done        = false;
    pickup_step_done     = false;

    flag_timer           = 0;
    temp_flag            = 0;
    hinder_counter       = 0;
    hinder_timer         = 0;

    init_karta();
}


long long current_time_ms() { //funktion för nuvarande tid
    struct timeval tv;      //sekunder & mikrosekunder
    gettimeofday(&tv, NULL);    //aktuell tid   
    return (long long)tv.tv_sec * 1000 + tv.tv_usec / 1000; //konvertera till ms
}

//loggfunktion för styrmodulen
void log_styr_response(const unsigned char *received) {
    FILE *f = fopen(VERIFY_LOG_FILE, "a");
    if (f == NULL) return;
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    fprintf(f, "[%02d:%02d:%02d] STYR SVAR (0x12): ", t->tm_hour, t->tm_min, t->tm_sec);
    for (int i = 0; i < PACKET_SIZE; i++) fprintf(f, "%02X ", received[i]);
    fprintf(f, "\n\n");
    fclose(f);
}

//uppdaterar aktivt och nästa beslut utifrån index och fas
void aktivt_beslut_fn(int index) {
    if (current_phase == PHASE_TO_ITEM) {
        aktivt_beslut = beslut_till_vara[index];
        if (aktivt_beslut == 'e' || aktivt_beslut == 'o') { //rotation
            nasta_beslut = 'f';                             //efter rotation ska 'f' skickas till styr
        } else if (aktivt_beslut == 'X') {                  //plocka vara
            nasta_beslut = pickup_cmd;
        } else {
            nasta_beslut = beslut_till_vara[index + 1];     
        }
    } else if (current_phase == PHASE_PICKUP) {
        aktivt_beslut = pickup_cmd;
        if (current_item_index + 1 < item_count) {      //finns flera varor att hämta
            nasta_beslut = 'f';
        } else {
            nasta_beslut = beslut_hem[0];           //inga varor kvar, använd beslut_hem array
        }
    } else if (current_phase == PHASE_TO_HOME) {
        aktivt_beslut = beslut_hem[index];
        if (aktivt_beslut == 'e' || aktivt_beslut == 'o') {
            nasta_beslut = 'f';
        } else {
            nasta_beslut = beslut_hem[index + 1];
        }
    }
}

//förbereder rotation eller startar en simulerad sträcka
static void prime_action_or_rotation(void) {
    if (aktivt_beslut == 'e' || aktivt_beslut == 'o') {
        is_rotating          = true;        //
        pending_rotation_cmd = aktivt_beslut;       //spara svängriktning
        aktivt_beslut        = 's';                 //skicka 's' så roboten stannar innan den roterar
        action_timer_start   = current_time_ms();   //starta timer för åtgärd
    } else if (sim_sensor) {
        sim_segment_timer = current_time_ms();
    }
}

//startar automatiserade uppdraget
void start_autonomous_sequence(unsigned char state) {
    if (item_count <= 0) {  //användaren måste välja en vara innan automatiska körningar startar
        printf("Inga varor placerade!\n");
        return;
    }

    current_item_index = 0;     
    vara_u = item_list_u[0];
    vara_v = item_list_v[0];

    printf("\nAutomatisk körning påbörjas, %d varor att hämta ===\n", item_count);
    printf("Vara 1/%d: väg %d <-> %d\n", item_count, vara_u, vara_v);
    planera_till_vara(START, 's');
    planera_hem_fran_pickup(99, 'a');

    current_auto_state   = state;
    current_phase        = PHASE_TO_ITEM;
    current_action_index = 0;
    korsning_aktiv       = 0;
    loop_counter         = 0;

    aktivt_beslut_fn(current_action_index);
    current_node = rutt_till_vara[0];
    current_dir  = 's';         //roboten står alltid söderut från start

    prime_action_or_rotation();

    log_next_action = true;
    route_changed   = true; //skickar rutten till gui:n
    printf("-> Rutt beräknad. Åker till vara 1/%d...\n", item_count);
    if (sim_sensor) printf("[SIMULERING] korsningar kommer upptäckas var %d ms\n", SIM_SEGMENT_MS);
}

//funktion för att initiera i2c
static int open_i2c(uint8_t addr, const char *name, bool *sim_flag) {
    int fd = open(I2C_DEVICE, O_RDWR);      //öppnar i2C bussen för läs & skriv
    if (fd >= 0) {                          //om bussen går att öppna
        ioctl(fd, I2C_SLAVE, addr);         //välj slav på bussen
        if (write(fd, NULL, 0) < 0) {       //kolla om får något svar
            *sim_flag = true;
            printf("[SIMULERING] %s (0x%02X) saknas. Avstängd.\n", name, addr);
        } else {
            printf("Uppkopplad till %s (0x%02X)\n", name, addr);
        }
    } else {
        *sim_flag = true;
        printf("[SIMULERING] kan inte öppna i2c bussen för %s.\n", name);
    }
    return fd;
}

static int setup_udp_socket(void) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port        = htons(UDP_PORT);

    if (bind(fd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }
    printf("Listening for UDP on port %d...\n\n", UDP_PORT);
    return fd;
}

//läser sensorpaket och extraherar information
static void read_sensors(void) {
    if (sim_sensor || i2c_sens_fd < 0) return;

    unsigned char sensor_raw[PACKET_SIZE];
    if (read(i2c_sens_fd, sensor_raw, PACKET_SIZE) != PACKET_SIZE) return;

    static unsigned char last_sensor_packet[PACKET_SIZE] = {0};
    if (memcmp(sensor_raw, last_sensor_packet, PACKET_SIZE) != 0) {
        log_sensor_data(sensor_raw);
        memcpy(last_sensor_packet, sensor_raw, PACKET_SIZE);
    }

    SensorData sd = parse_sensor_packet(sensor_raw);
    flags      = sd.flags;      
    line_var_f = sd.line_var_f;  
    line_var_b = sd.line_var_b;
    angle      = sd.angle;
    gyro1      = sd.gyro1;
    gyro2      = sd.gyro2;

    flags_korsning = (flags & 0x0C) >> 2;   //typ av korsning 
    flags_ir       = (flags & 0x10) >> 4;   //om ir sensorn upptäcker ett hinder

    if (flags_ir == 0b00000001) {
        is_hinder = true;       //sätt till true om flaggan är aktiv
    }

    if (flags_korsning == 1)      pickup_cmd = 'v';     //1 betyder upphämtningplats till vänster
    else if (flags_korsning == 3) pickup_cmd = 'h';     //3 betyder upphämtningsplats åt höger
}

//läser status från styrmodulen
static void read_styr(void) {
    if (sim_motor || i2c_styr_fd < 0) return;

    static long long last_styr_read_ms = 0;
    long long now = current_time_ms();
    if (now - last_styr_read_ms < 200) return;  //vi vill inte läsa för ofta 
    last_styr_read_ms = now;

    unsigned char styr_raw[PACKET_SIZE];
    if (read(i2c_styr_fd, styr_raw, PACKET_SIZE) != PACKET_SIZE) return;

    static unsigned char last_styr_packet[PACKET_SIZE] = {0};
    if (memcmp(styr_raw, last_styr_packet, PACKET_SIZE) != 0) { //logga varje ändring
        log_styr_response(styr_raw);
        memcpy(last_styr_packet, styr_raw, PACKET_SIZE);
    }

    StyrResponse resp = parse_styr_response(styr_raw, PACKET_SIZE);
    styr_gas_right = resp.gas_right;
    styr_gas_left  = resp.gas_left;
    styr_claw_r    = resp.claw_pos_r;
    styr_claw_z    = resp.claw_pos_z;
  
    if (resp.action_done == 1) {    //om åtgärd klar
        action_done = 1;            //sätt flagga till 1
    }
}
//hanterar paket från gui:n
static void handle_command_packet(const CommandPacket *cmd) {
    if (cmd->state == 0x00 || cmd->state == 0x01) { //om hjulen skall köras automatiskt
        if (cmd->action == 'f' && current_phase == PHASE_IDLE) { //börja automatiska styrningen om användaren skickar 'f'
            start_autonomous_sequence(cmd->state);
            return;
        }
        unsigned char fwd[PACKET_SIZE];
        build_motor_packet(fwd, cmd->state, false, cmd->action,
                           line_var_f, line_var_b, gyro1, gyro2);
        fwd[2] = cmd->target;
        if (!sim_motor) write(i2c_styr_fd, fwd, PACKET_SIZE);   //skicka packet till styr
        log_verification(fwd, cmd->action);

    }
    else if (cmd->state == 0x02 || cmd->state == 0x03) {    //om vi byter till manuell hjulstyrning
        init_karta();       //initiera kartan igen för att ta bort hinder
        is_hinder2 = false; //lås upp hinderspärr
        if (current_phase != PHASE_IDLE) {
            printf("\nManuellt läge! Avbryter automatisk körning.\n");
            current_phase        = PHASE_IDLE;
            is_rotating          = false;
            is_picking_up        = false;
            is_dropping          = false;
            current_action_index = 0;
            korsning_aktiv       = 0;
            aktivt_beslut        = 's';
            nasta_beslut         = 's';
        }
        unsigned char fwd[PACKET_SIZE];
        build_motor_packet(fwd, cmd->state, false, cmd->action,
                           line_var_f, line_var_b, gyro1, gyro2);       //skicka packet till styr
        fwd[2] = cmd->target;
        if (!sim_motor) write(i2c_styr_fd, fwd, PACKET_SIZE);
        log_verification(fwd, cmd->action);
    }
}

//tar emot varulista från gui:n
static void handle_item_list_packet(const ItemListPacket *items) {
    item_count = items->count;
    memcpy(item_list_u, items->items_u, items->count * sizeof(uint8_t));
    memcpy(item_list_v, items->items_v, items->count * sizeof(uint8_t));
    current_item_index = 0;
    printf("[VAROR] Mottagit %d varor från GUI\n", item_count);
}

static void handle_network_packets(void) {
    unsigned char buffer[BUFFER_SIZE];
    socklen_t len = sizeof(cliaddr);
    int n = recvfrom(sockfd, buffer, BUFFER_SIZE, MSG_DONTWAIT,
                     (struct sockaddr *)&cliaddr, &len);
    if (n <= 0) return;

    gui_known = true;

    if (n == PACKET_SIZE && buffer[0] == 0x09) {
        printf("\n=== RESET RECEIVED FROM GUI ===\n\n");
        reset();

        unsigned char stop_pkt[PACKET_SIZE];
        build_motor_packet(stop_pkt, 0x03, false,
                           's', line_var_f, line_var_b, gyro1, gyro2);
        if (!sim_motor) write(i2c_styr_fd, stop_pkt, PACKET_SIZE);

        // Clear item list
        item_count = 0;
        current_item_index = 0;
        return;
    }

    CommandPacket cmd = parse_command_packet(buffer, n);
    if (cmd.valid) {
        handle_command_packet(&cmd);
    }

    ItemListPacket items = parse_item_list_packet(buffer, n);
    if (items.valid && items.count > 0) {
        handle_item_list_packet(&items);
    }
}

//blockera väg som hindret befinner sig på
static void block_edge_ahead(void) {
    if (current_node == START) {
        vag[0][5] = 0;
        vag[5][0] = 0;
    }
    else if (current_dir == 'n') {
        vag[current_node - 5][current_node - 10] = 0;
        vag[current_node - 10][current_node - 5] = 0;
    }
    else if (current_dir == 'e') {
        vag[current_node + 1][current_node + 2] = 0;
        vag[current_node + 2][current_node + 1] = 0;
    }
    else if (current_dir == 's') {
        vag[current_node + 5][current_node + 10] = 0;
        vag[current_node + 10][current_node + 5] = 0;
    }
    else if (current_dir == 'w') {
        vag[current_node - 1][current_node - 2] = 0;
        vag[current_node - 2][current_node - 1] = 0;
    }
}
//hinderhantering
static void handle_obstacle(void) {
    if (!(is_hinder && !is_hinder2 && (hinder_counter < 5) && (hinder_timer > SLEEP/50))) {    //om inte mer än 5 hinder och timern hunnit gå ett tag
        return;
    }

    block_edge_ahead();
    current_action_index = 0;   //reseta action_index eftersom beslutsarrayena uppdateras

    if (current_phase == PHASE_TO_ITEM && current_item_index == 0) {
        planera_till_vara(current_node, current_dir);
    } else if (current_phase == PHASE_TO_ITEM && current_item_index > 0) {
        planera_nasta_vara(current_node, current_dir);
    } else if (current_phase == PHASE_TO_HOME) {
        planera_hem_fran_pickup(current_node, current_dir);
    }
    aktivt_beslut_fn(current_action_index);

    is_hinder       = false;
    is_hinder2      = true;
    route_changed   = true;
    log_next_action = true;

    hinder_counter++;
    hinder_timer = 0;
}

//kollar om åtgärden är klar
static bool poll_action_done(long long elapsed_in_state, bool *done_flag) {
    if (*done_flag) return true;
    if (sim_motor || elapsed_in_state <= 300) return false;

    if (action_done == 1) {
        action_done = 0; 
        *done_flag = true;
        return true;
    }
    return false;
}

//hanterar rotationer
static void handle_rotation_state(long long elapsed_in_state) {
    flag_timer = 0;

    if (sim_motor) {
        rotation_done = ((aktivt_beslut == 's' || aktivt_beslut == 'z') && elapsed_in_state >= 1000) ||
                        ((aktivt_beslut == 'e' || aktivt_beslut == 'o') && elapsed_in_state >= 2000);
    } else {
        poll_action_done(elapsed_in_state, &rotation_done); //om rotation_done är true resetas action_done
    }

    if (rotation_done && (aktivt_beslut == 's' || aktivt_beslut == 'z')) {  //om vi inte roterar jsut nu och vi redan har stoppat eller backstoppat
        aktivt_beslut      = pending_rotation_cmd;  //lagra kommandot
        rotation_done      = false;
        action_timer_start = current_time_ms();
        log_next_action    = true;
    }
    else if (rotation_done && (aktivt_beslut == 'e' || aktivt_beslut == 'o')) { //om rotationen är klar
        is_rotating    = false;
        aktivt_beslut  = 'f';
        rotation_done  = false;
        korsning_aktiv = 1; //undviker att samma korsning aktiveras igen

        //uppdatera rätt beslutsarray
        if (current_phase == PHASE_TO_ITEM) {   
            nasta_beslut = beslut_till_vara[current_action_index + 1];
        } else if (current_phase == PHASE_TO_HOME) {
            nasta_beslut = beslut_hem[current_action_index + 1];
        }
        log_next_action = true;

        if (sim_sensor) sim_segment_timer = current_time_ms();
    }
}

//procedur efter pickup
static void start_phase_after_pickup(void) {
    if (current_item_index < item_count) {  //kollar om det finns fler varor
        vara_u = item_list_u[current_item_index];   
        vara_v = item_list_v[current_item_index];
        printf("\n-> Vara %d/%d: väg %d <-> %d\n",
               current_item_index + 1, item_count, vara_u, vara_v); 
        planera_nasta_vara(99, 'a');    //skicka '99' och 'a' enligt standard
        planera_hem_fran_pickup(99, 'a');

        current_phase        = PHASE_TO_ITEM; 
        current_action_index = 0;
        aktivt_beslut_fn(current_action_index);
        prime_action_or_rotation();

        printf("-> Fasbyte: Åker till vara %d/%d...\n",
               current_item_index + 1, item_count);
    } else {  //åk hem
        planera_hem_fran_pickup(99, 'a');
        current_phase        = PHASE_TO_HOME
        current_action_index = 0;
        aktivt_beslut_fn(current_action_index);
        prime_action_or_rotation();

        printf("\n-> Fasbyte: Alla %d varor är upphämtade. Åker hem...\n", item_count);
    }
    log_next_action = true;
    route_changed   = true;
}

//procedur för att hämta vara 
static void handle_pickup_state(long long elapsed_in_state) {
    flag_timer = 0;     //nollställ timer när vi plockar en vara så att korsningen roboten står i inte detekteras igen

    if (sim_motor) {
        pickup_step_done = (aktivt_beslut == 'x' && elapsed_in_state >= 1500) ||
                           (aktivt_beslut == 'v' && elapsed_in_state >= 3000);
    } else {
        poll_action_done(elapsed_in_state, &pickup_step_done);
    }

    if (pickup_step_done && aktivt_beslut == 'x') { //roboten har stannat och vi vill uppdatera beslut
        aktivt_beslut      = pickup_cmd;
        pickup_step_done   = false;
        action_timer_start = current_time_ms();
       
        if (current_item_index + 1 < item_count){   //åka vidare till nästa vara eller hem
            nasta_beslut = 'f';
        }else{
            nasta_beslut = beslut_hem[0];
        }
        log_next_action = true;

    }
    else if (pickup_step_done && (aktivt_beslut == 'v' || aktivt_beslut == 'h')) { //roboten har plockat vara och vi vill gå vidare
        is_picking_up    = false;
        current_item_index++;
        pickup_step_done = false;
        start_phase_after_pickup();
    }
}

//procedur för att lämna vara
static void handle_drop_state(long long elapsed_in_state) {
    flag_timer = 0;     //reseta räknare för korsningsflagga så länge vi lämnar vara

    if (sim_motor) {
        drop_step_done = ((aktivt_beslut == 's' || aktivt_beslut == 'z') && elapsed_in_state >= 1500) ||
                         (aktivt_beslut == 'w' && elapsed_in_state >= 3000);
    } else {
        poll_action_done(elapsed_in_state, &drop_step_done);
    }

    if (drop_step_done && (aktivt_beslut == 's' || aktivt_beslut == 'z')) {     //roboten har stannat och vi vill uppdatera beslut
        aktivt_beslut      = 'w';
        drop_step_done     = false;
        action_timer_start = current_time_ms();
        nasta_beslut       = 's';
        log_next_action    = true;
    }
    else if (drop_step_done && aktivt_beslut == 'w') {
        reset();
        printf("\nUPPDRAGET KLART!\n\n");

        unsigned char stop_pkt[PACKET_SIZE];
        build_motor_packet(stop_pkt, current_auto_state, false,
                           's', line_var_f, line_var_b, gyro1, gyro2);
        if (!sim_motor) write(i2c_styr_fd, stop_pkt, PACKET_SIZE);
    }
}

//funktion som avgör om ny korsning ska triggas
static bool detect_intersection(void) {
    if (sim_sensor) {
        if (sim_segment_timer > 0 && (current_time_ms() - sim_segment_timer) >= SIM_SEGMENT_MS) {
            sim_segment_timer = 0;
            return true;
        }
        return false;
    }

    if (flags_korsning != temp_flag) {  //om flags_korsning inte är samma som temp_flag
        if (flag_timer > SLEEP/1500) {  //flag_timer måste uppnått ett visst värde så vi inte triggar samma korsning igen
            bool real_intersection = (flags_korsning == 2);     //korsning (ej upphämtningsplats)
            bool pickup_marker     = ((flags_korsning == 1 || flags_korsning == 3) && nasta_beslut == 'X'); //om nästa beslut är 'X' och flaggorna visar upphämtningsplats, aktivera pickup
            if (real_intersection || pickup_marker) {   //om någon av korsningarna godkänns
                flags_ny_korsning = 1;  //uppdatera flagga
            }
        }
        temp_flag = flags_korsning;     //spara korsningstyp
    }

    bool triggered = false;
    if (flags_ny_korsning && !korsning_aktiv) { //om ny korsning upptcäks och vi inte står i en korsning
        triggered         = true;
        korsning_aktiv    = 1;
        flags_ny_korsning = 0;
    }

    if (flags_korsning == 0) {
        korsning_aktiv = 0;
    }
    return triggered;
}
//hantera kornsing när den triggats
static void handle_intersection(void) {
    char previous_action = aktivt_beslut; //spara beslutet så vi kan använda det senare

    current_action_index++;
    aktivt_beslut_fn(current_action_index); 
    action_timer_start = current_time_ms();     //börja timer för åtgärd


    if (current_phase == PHASE_TO_ITEM) {  
        current_node = rutt_till_vara[current_action_index];        //uppdatera aktuell nod
        if (rutt_till_vara[current_action_index + 1] != STOP) {
            current_dir = nodriktningsmatris[rutt_till_vara[current_action_index]]
                                            [rutt_till_vara[current_action_index + 1]];
        }
    } else if (current_phase == PHASE_TO_HOME) {    
        current_node = rutt_hem[current_action_index];
        if (rutt_hem[current_action_index + 1] != STOP) {
            current_dir = nodriktningsmatris[rutt_hem[current_action_index]]
                                            [rutt_hem[current_action_index + 1]];
        }
    }

    if (aktivt_beslut == 'e' || aktivt_beslut == 'o') {
        is_rotating          = true;
        pending_rotation_cmd = aktivt_beslut;
        if (previous_action == 'b') {
            aktivt_beslut = 'z';        //kommando för att roboten ska stanna när den backar (då sensorerna ligger för långt fram)
        } else {
            aktivt_beslut = 's';
        }
        action_timer_start   = current_time_ms();
        log_next_action      = true;
    }
    else if (aktivt_beslut == 'X') {
        if (current_phase == PHASE_TO_ITEM) {
            current_phase   = PHASE_PICKUP;
            aktivt_beslut   = 'x';
            nasta_beslut    = pickup_cmd;
            is_picking_up   = true;
            printf("\n-> Plockar vara %d/%d...\n", current_item_index + 1, item_count);
            log_next_action = true;
        }
        else if (current_phase == PHASE_TO_HOME) {
            current_phase      = PHASE_DROP;
            if (previous_action == 'b') {
                aktivt_beslut = 'z';        //kommando för att roboten ska stanna när den backar (då sensorerna ligger för långt fram)
            } else {
                aktivt_beslut = 's';
            }
            nasta_beslut       = 'w';
            is_dropping        = true;
            action_timer_start = current_time_ms();
            printf("\n-> Lämnar vara\n");
            log_next_action    = true;
        }
    }
    else {
        log_next_action = true;
    }

    if (sim_sensor && !is_rotating && !is_picking_up && !is_dropping && aktivt_beslut != 'X') {
        sim_segment_timer = current_time_ms();
    }
}

//bygger och skickar paket till styr
static void send_motor_command(void) {
    if (current_phase == PHASE_IDLE || aktivt_beslut == 'X') return;

    bool pickup_flag = (current_phase == PHASE_PICKUP && (aktivt_beslut == 'v' || aktivt_beslut == 'h')) ||
                       (current_phase == PHASE_DROP   &&  aktivt_beslut == 'w');       //flagga för plockande av vara

    unsigned char auto_packet[PACKET_SIZE];
    build_motor_packet(auto_packet, current_auto_state, pickup_flag,
                       aktivt_beslut, line_var_f, line_var_b, gyro1, gyro2);

    if (!sim_motor) write(i2c_styr_fd, auto_packet, PACKET_SIZE);

    if (log_next_action) {
        printf("Beslut uppdaterat till: '%c' (Skickar till styr: '%c', Index: %d, Nästa: '%c')\n",
               aktivt_beslut, auto_packet[3], current_action_index, nasta_beslut);
        log_next_action = false;
    }

    static int blasting_log_counter = 0;     //spammar loggen med packetinformation
    blasting_log_counter++;
    if (blasting_log_counter >= 50) {
        log_verification(auto_packet, auto_packet[3]);
        blasting_log_counter = 0;
    }
}

//ett tick i den autonoma tillståndsmaskinen
static void run_autonomous_tick(void) {
    if (current_phase == PHASE_IDLE) return;

    long long elapsed_in_state = current_time_ms() - action_timer_start;

    handle_obstacle();  //hantera alltid hinder först

    if (is_rotating) {  //annars kolla om vi roterar
        handle_rotation_state(elapsed_in_state);
    }
    else if (is_picking_up) {   //annars kolla om vi ska ta upp vara
        handle_pickup_state(elapsed_in_state);
    }
    else if (is_dropping) {     //annars kolal om vi ska lämna vara
        handle_drop_state(elapsed_in_state);
    }
    else if (detect_intersection()) {   //om inget, kolla om korsning triggas
        handle_intersection();
    }

    send_motor_command();   //skicka till styr
}

static void send_telemetry(void) {
    if (!gui_known) return;
    telemetry_counter++;
    if (telemetry_counter < 10) return;

    unsigned char tpkt[PACKET_SIZE + 10];
    build_telemetry_packet(tpkt,
                           (uint8_t)current_phase, aktivt_beslut, nasta_beslut,
                           line_var_f, gyro1, gyro2, flags, current_node,
                           (uint8_t)current_item_index, (uint8_t)item_count,
                           current_dir, action_done,
                           styr_gas_right, styr_gas_left, styr_claw_r, styr_claw_z);
    sendto(sockfd, tpkt, (PACKET_SIZE + 10), 0,
           (struct sockaddr *)&cliaddr, sizeof(cliaddr));
    telemetry_counter = 0;
}

static void send_route_update(void) {
    if (!gui_known || !route_changed) return;
    route_changed = false;

    int *rutt = (current_phase == PHASE_TO_HOME) ? rutt_hem : rutt_till_vara;
    unsigned char rpkt[NODES + 3];
    int rpkt_len = build_route_packet(rpkt, rutt, NODES);
    sendto(sockfd, rpkt, rpkt_len, 0,
           (struct sockaddr *)&cliaddr, sizeof(cliaddr));
}

//huvudprogrammet
int main() {
    init_karta(); 

    FILE *clr = fopen(VERIFY_LOG_FILE, "w");
    if (clr) fclose(clr);
    printf("--- PI CORE: DUAL I2C (0x10 & 0x12) + UDP ROUTER ---\n");

    i2c_styr_fd = open_i2c(STYRKOMM_ADDR, "Motor Controller", &sim_motor);
    i2c_sens_fd = open_i2c(SENSOR_ADDR,   "Sensor Board",     &sim_sensor);
    if (sim_sensor) {
        printf("[SIM] Using time-based intersection simulation (%d ms).\n", SIM_SEGMENT_MS);
    }
    if (sim_sensor || sim_motor) {
        printf("\n*** RUNNING IN SIM MODE ***\n\n");
    }

    sockfd = setup_udp_socket();

   
    while (1) {
        read_sensors();
        read_styr();
        handle_network_packets();
        run_autonomous_tick();
        send_telemetry();
        send_route_update();

        flag_timer++;
        hinder_timer++;
        usleep(SLEEP);  //detta ger att programmet körs i ungefär 40 Hz (max 40 Hz)
    }

    close(sockfd);
    if (i2c_styr_fd >= 0) close(i2c_styr_fd);
    if (i2c_sens_fd >= 0) close(i2c_sens_fd);
    return 0;
}