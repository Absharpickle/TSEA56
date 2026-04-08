#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string.h>

void log_comparison(uint8_t* sent, uint8_t* received) {
    FILE *f = fopen("validering.txt", "a");
    if (f == NULL) return;

    fprintf(f, "SKICKAT:  ");
    for(int i=0; i<8; i++) fprintf(f, "%02X ", sent[i]);
    
    fprintf(f, "\nMOTTAGET: ");
    for(int i=0; i<8; i++) fprintf(f, "%02X ", received[i]);

    if (memcmp(sent, received, 8) == 0) {
        fprintf(f, " | STATUS: MATCH ✔️\n\n");
    } else {
        fprintf(f, " | STATUS: FEL ❌\n\n");
    }
    fclose(f);
}

int main() {
    int file;
    uint8_t buffer_from_sensor[8];
    uint8_t buffer_back_from_mottagare[8];

    if ((file = open("/dev/i2c-1", O_RDWR)) < 0) return 1;

    while (1) {
        // 1. LÄS FRÅN SENSOR (0x11)
        ioctl(file, I2C_SLAVE, 0x11);
        if (read(file, buffer_from_sensor, 8) == 8) {
            
            // 2. SKRIV TILL MOTTAGARE (0x12)
            ioctl(file, I2C_SLAVE, 0x12);
            if (write(file, buffer_from_sensor, 8) == 8) {
                
                // 3. LÄS TILLBAKA FRÅN MOTTAGARE (0x12)
                usleep(5000); // Liten paus så AVR hinner förbereda ISR
                if (read(file, buffer_back_from_mottagare, 8) == 8) {
                    printf("Verifikation klar. Kolla validering.txt\n");
                    log_comparison(buffer_from_sensor, buffer_back_from_mottagare);
                }
            }
        }
        usleep(100000); 
    }
    return 0;
}