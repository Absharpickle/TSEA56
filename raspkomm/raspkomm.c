#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

int main() {
    int file;
    uint8_t buffer[8];

    if ((file = open("/dev/i2c-1", O_RDWR)) < 0) return 1;

    printf("Startar Kedjan: Sensor (0x11) -> Pi -> Mottagare (0x12)...\n");

    while (1) {
        // --- STEG 1: LÄS FRÅN SENSOR ---
        ioctl(file, I2C_SLAVE, 0x11);
        if (read(file, buffer, 8) == 8) {
            
            // --- STEG 2: OM LÄSNING LYCKADES, SKICKA VIDARE TILL MOTTAGARE ---
            ioctl(file, I2C_SLAVE, 0x12);
            if (write(file, buffer, 8) == 8) {
                printf("Paket dirigerat framgångsrikt!\n");
            } else {
                printf("Kunde läsa från sensor, men mottagaren svarade inte (NACK).\n");
            }
            
        } else {
            printf("Sensorn (0x11) svarade inte.\n");
        }
        
        usleep(100000); // 100ms paus (10 Hz uppdateringsfrekvens)
    }
    return 0;
}