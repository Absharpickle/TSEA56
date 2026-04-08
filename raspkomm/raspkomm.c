#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h> // För tidstämplar

// Funktion för att skriva till loggfilen
void log_to_file(uint8_t* data, const char* status) {
    FILE *f = fopen("logg.txt", "a"); // Öppna i "append"-läge
    if (f == NULL) return;

    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    
    // Skriv tidstämpel och status
    fprintf(f, "[%02d:%02d:%02d] %s: ", t->tm_hour, t->tm_min, t->tm_sec, status);
    
    // Skriv datan i hex-format
    for (int i = 0; i < 8; i++) {
        fprintf(f, "%02X ", data[i]);
    }
    fprintf(f, "\n");
    fclose(f);
}

int main() {
    int file;
    uint8_t buffer[8];

    if ((file = open("/dev/i2c-1", O_RDWR)) < 0) return 1;

    printf("Loggning startad. Se logg.txt för data...\n");

    while (1) {
        // --- STEG 1: LÄS FRÅN SENSOR (0x11) ---
        ioctl(file, I2C_SLAVE, 0x11);
        if (read(file, buffer, 8) == 8) {
            
            // --- STEG 2: SKICKA TILL MOTTAGARE (0x12) ---
            ioctl(file, I2C_SLAVE, 0x12);
            
            // Fixade buggen: använde 'buffer' istället för 'buf'
            if (write(file, buffer, 8) == 8) {
                printf("Data loggad och skickad!\n");
                log_to_file(buffer, "OK (Frammatat)");
            } else {
                printf("Mottagaren (0x12) gav NACK.\n");
                log_to_file(buffer, "FEL (NACK från 0x12)");
            }
            
        } else {
            // Logga även om sensorn inte svarar för felsökning
            uint8_t empty[8] = {0};
            log_to_file(empty, "FEL (Sensor 0x11 svarar ej)");
        }
        
        usleep(100000); // 100ms paus
    }
    return 0;
}