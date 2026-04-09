#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <termios.h>

#define SLAVE_ADDRESS 0x12

// Funktion för att läsa tangenttryckningar utan att trycka Enter
int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

// Funktion för att skriva status till fil
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
    printf("Status sparad till 'klo_status.txt'\n");
}

int main() {
    int file;
    if ((file = open("/dev/i2c-1", O_RDWR)) < 0) {
        printf("Fel: Kunde inte öppna I2C.\n");
        return 1;
    }

    uint8_t buffer_out[8] = {0x05, 0, 0, 0, 0, 0, 0, 0xFF};
    bool manuellt_lage_aktivt = false;
    int aktuellt_lage = 1; // Standardläge: 1 (X/Y)

    printf("--- MANUELL KONTROLL (raspclaw) ---\n");
    printf("Tryck [SPACE] för att starta/avsluta manuellt läge.\n");

    while (1) {
        if (kbhit()) {
            int key = getchar();

            // --- Hantera SPACE (Starta/Avsluta) ---
            if (key == ' ') {
                manuellt_lage_aktivt = !manuellt_lage_aktivt;
                if (manuellt_lage_aktivt) {
                    printf("\n[MANUELLT LÄGE AKTIVERAT]\n");
                    printf("LÄGE 1: X/Y (Pilar)\n");
                } else {
                    printf("\n[MANUELLT LÄGE AVSLUTAT] Hämtar status...\n");
                    
                    // Skicka kommando 5 för att be om status
                    buffer_out[1] = 5; 
                    ioctl(file, I2C_SLAVE, SLAVE_ADDRESS);
                    write(file, buffer_out, 8);
                    
                    usleep(10000); // Ge AVR tid att byta till TX-läge
                    
                    uint8_t status_in[8];
                    if (read(file, status_in, 8) == 8) {
                        spara_status_till_fil(status_in);
                    } else {
                        printf("Kunde inte läsa status från AVR.\n");
                    }
                    break; // Avsluta programmet
                }
            }

            if (!manuellt_lage_aktivt) continue;

            // --- Byta läge (1, 2, 3, 4) ---
            if (key == '1') { aktuellt_lage = 1; printf("LÄGE 1: X/Y-styrning\n"); }
            if (key == '2') { aktuellt_lage = 2; printf("LÄGE 2: Vippa (Upp/Ned)\n"); }
            if (key == '3') { aktuellt_lage = 3; printf("LÄGE 3: Rotation (Vänster/Höger)\n"); }
            if (key == '4') { aktuellt_lage = 4; printf("LÄGE 4: Gripklo (Upp=Grip, Ned=Släpp)\n"); }

            // --- Hantera piltangenter (Esc-sekvenser i Linux: 27, 91, [65-68]) ---
            if (key == 27) { 
                getchar(); // Fånga '['
                int arrow = getchar(); // Fånga 'A' (Upp), 'B' (Ned), 'C' (Höger), 'D' (Vänster)

                buffer_out[1] = aktuellt_lage; // Vilket system vill vi styra?
                buffer_out[2] = 0; // Återställ värde

                if (aktuellt_lage == 1) { // X/Y
                    if (arrow == 'A') buffer_out[2] = 1;       // +Y (Upp)
                    else if (arrow == 'B') buffer_out[2] = -1; // -Y (Ned)
                    else if (arrow == 'C') buffer_out[2] = 2;  // +X (Höger) (Använder '2' för att skilja på Y och X)
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

                // Skicka paketet om en pil trycktes
                if (arrow >= 'A' && arrow <= 'D') {
                    ioctl(file, I2C_SLAVE, SLAVE_ADDRESS);
                    write(file, buffer_out, 8);
                    // printf("Skickade Kommando: Läge %d, Värde %d\n", buffer_out[1], (int8_t)buffer_out[2]);
                }
            }
        }
        usleep(10000); // 10ms CPU-vila
    }

    close(file);
    return 0;
}