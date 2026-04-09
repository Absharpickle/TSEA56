#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <termios.h>

#define SLAVE_ADDRESS 0x12

// =================================================================
// TERMINAL-HANTERING (RAW MODE) FÖR PILTANGENTER
// =================================================================
struct termios orig_termios;

void disable_raw_mode() {
    // Återställ terminalen till normalt läge när programmet avslutas
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
}

void enable_raw_mode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(disable_raw_mode); // Se till att disable_raw_mode körs vid exit
    
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON); // Stäng av text-eko och krav på "Enter"
    raw.c_cc[VMIN] = 0;              // Läs direkt (non-blocking)
    raw.c_cc[VTIME] = 0;             // Ingen timeout
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

// =================================================================
// HJÄLPFUNKTIONER
// =================================================================
void spara_status_till_fil(uint8_t* status_data) {
    FILE *f = fopen("klo_status.txt", "w");
    if (f == NULL) return;

    fprintf(f, "--- SENASTE STATUS FRÅN ROBOT ---\n");
    fprintf(f, "X-position: %d\n", (int8_t)status_data[1]);
    fprintf(f, "Y-position: %d\n", (int8_t)status_data[2]);
    fprintf(f, "Vippa:      %d\n", (int8_t)status_data[3]);
    fprintf(f, "Rotation:   %d\n", (int8_t)status_data[4]);
    fprintf(f, "Klo-status: %s\n", (status_data[5] == 1) ? "Stängd" : "Öppen");
    fclose(f);
    printf("\n\rStatus sparad till 'klo_status.txt'\n\r");
}

// =================================================================
// HUVUDPROGRAM
// =================================================================
int main() {
    int file;
    if ((file = open("/dev/i2c-1", O_RDWR)) < 0) {
        printf("Fel: Kunde inte öppna I2C-bussen.\n");
        return 1;
    }

    uint8_t buffer_out[8] = {0x05, 0, 0, 0, 0, 0, 0, 0xFF};
    bool manuellt_lage_aktivt = false;
    int aktuellt_lage = 1; // Standardläge: 1 (X/Y)

    // Sätt terminalen i Raw Mode för att fånga piltangenterna perfekt
    enable_raw_mode();

    // Notera: I Raw Mode använder vi \r\n istället för bara \n för snygg radbrytning
    printf("--- MANUELL KONTROLL (raspclaw) ---\n\r");
    printf("Tryck [SPACE] för att starta/avsluta manuellt läge.\n\r");

    char c;
    while (1) {
        // Läs in EN byte från tangentbordet
        if (read(STDIN_FILENO, &c, 1) == 1) {
            
            // --- Hantera SPACE (Starta/Avsluta) ---
            if (c == ' ') {
                manuellt_lage_aktivt = !manuellt_lage_aktivt;
                if (manuellt_lage_aktivt) {
                    printf("\n\r[MANUELLT LÄGE AKTIVERAT]\n\r");
                    printf("LÄGE 1: X/Y (Pilar)\n\r");
                } else {
                    printf("\n\r[MANUELLT LÄGE AVSLUTAT] Hämtar status...\n\r");
                    
                    // Skicka kommando 5 för att be om status
                    buffer_out[1] = 5; 
                    ioctl(file, I2C_SLAVE, SLAVE_ADDRESS);
                    write(file, buffer_out, 8);
                    
                    usleep(10000); // Ge AVR tid att byta till TX-läge
                    
                    uint8_t status_in[8] = {0};
                    if (read(file, status_in, 8) == 8) {
                        spara_status_till_fil(status_in);
                    } else {
                        printf("Kunde inte läsa status från AVR.\n\r");
                    }
                    break; // Avsluta while-loopen och programmet
                }
            }

            if (!manuellt_lage_aktivt) continue;

            // --- Byta läge (1, 2, 3, 4) ---
            if (c == '1') { aktuellt_lage = 1; printf("LÄGE 1: X/Y-styrning\n\r"); }
            if (c == '2') { aktuellt_lage = 2; printf("LÄGE 2: Vippa (Upp/Ned)\n\r"); }
            if (c == '3') { aktuellt_lage = 3; printf("LÄGE 3: Rotation (Vänster/Höger)\n\r"); }
            if (c == '4') { aktuellt_lage = 4; printf("LÄGE 4: Gripklo (Upp=Grip, Ned=Släpp)\n\r"); }

            // --- Hantera piltangenter (Escape-sekvens: \x1b -> [ -> A/B/C/D) ---
            if (c == '\x1b') { 
                char seq[2];
                // Läs de nästkommande 2 byten ( [ och Bokstaven )
                if (read(STDIN_FILENO, &seq[0], 1) == 1 && read(STDIN_FILENO, &seq[1], 1) == 1) {
                    if (seq[0] == '[') {
                        char arrow = seq[1];

                        buffer_out[1] = aktuellt_lage; // Vilket system vill vi styra?
                        buffer_out[2] = 0; // Återställ värde

                        if (aktuellt_lage == 1) { // X/Y
                            if (arrow == 'A') buffer_out[2] = 1;       // +Y (Upp)
                            else if (arrow == 'B') buffer_out[2] = -1; // -Y (Ned)
                            else if (arrow == 'C') buffer_out[2] = 2;  // +X (Höger)
                            else if (arrow == 'D') buffer_out[2] = -2; // -X (Vänster)
                        }
                        else if (aktuellt_lage == 2) { // Vippa
                            if (arrow == 'A') buffer_out[2] = 1;       // Upp
                            else if (arrow == 'B') buffer_out[2] = -1; // Ned
                        }
                        else if (aktuellt_lage == 3) { // Rotation
                            if (arrow == 'C') buffer_out[2] = 1;       // + (Höger)
                            else if (arrow == 'D') buffer_out[2] = -1; // - (Vänster)
                        }
                        else if (aktuellt_lage == 4) { // Gripklo
                            if (arrow == 'A') buffer_out[2] = 1;       // Grip (Upp)
                            else if (arrow == 'B') buffer_out[2] = 0;  // Släpp (Ned)
                        }

                        // Skicka paketet till AVR
                        ioctl(file, I2C_SLAVE, SLAVE_ADDRESS);
                        write(file, buffer_out, 8);
                        
                        // Avkommentera raden nedan om du vill se en utskrift varje gång du klickar på en pil
                        // printf("-> Skickade Kommando: Läge %d, Värde %d\n\r", buffer_out[1], (int8_t)buffer_out[2]);
                    }
                }
            }
        }
        usleep(10000); // 10ms CPU-vila (Minskar CPU-användningen till nästan noll)
    }

    close(file);
    return 0; // Raw mode stängs automatiskt av här tack vare atexit()
}