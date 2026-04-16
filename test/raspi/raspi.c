#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <string.h>

/*
 * Förväntat paketformat från Sensormodulen (8 bytes):
 * [0]: status (bitmask)
 * [1-2]: dev0 (int16_t) - Linjeavvikelse sensorrad 0
 * [3-4]: dev1 (int16_t) - Linjeavvikelse sensorrad 1
 * [5]: dist (uint8_t)   - IR-avstånd
 * [6-7]: omega (int16_t)- Gyrovärde
 */

int main() {
    int file;
    const char *device = "/dev/i2c-1";
    int addr = 0x10;
    uint8_t buffer[8];

    // 1. Öppna bussen
    if ((file = open(device, O_RDWR)) < 0) {
        fprintf(stderr, "FEL: Kunde inte öppna %s. %s\n", device, strerror(errno));
        return 1;
    }

    // 2. Sätt slav-adress
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        fprintf(stderr, "FEL: Kunde inte ansluta till slav 0x%02X. %s\n", addr, strerror(errno));
        close(file);
        return 1;
    }

    printf("--- I2C MONITOR STARTAD (Slav 0x%02X) ---\n", addr);
    printf("Status |  Dev0 |  Dev1 | Dist |  Gyro | Rådata (HEX)\n");
    printf("----------------------------------------------------\n");

    while (1) {
        // 3. Läs data
        int bytes_read = read(file, buffer, 8);
        
        if (bytes_read == 8) {
            // Tolka Little Endian (samma som i din AVR-kod)
            uint8_t stat  = buffer[0];
            int16_t dev0  = (int16_t)(buffer[1] | (buffer[2] << 8));
            int16_t dev1  = (int16_t)(buffer[3] | (buffer[4] << 8));
            uint8_t dist  = buffer[5];
            int16_t omega = (int16_t)(buffer[6] | (buffer[7] << 8));

            // Printa formaterat i terminalen
            printf(" 0x%02X  | %5d | %5d | %4u | %5d | ", 
                   stat, dev0, dev1, dist, omega);
            
            // Printa även rådata i HEX för felsökning
            for(int i=0; i<8; i++) {
                printf("%02X ", buffer[i]);
            }
            printf("\n");

        } else if (bytes_read < 0) {
            fprintf(stderr, "\nLäsfel: %s\n", strerror(errno));
            // Om bussen hänger sig kan man behöva vänta lite innan nytt försök
            sleep(1);
        } else {
            printf("\nVarning: Läste bara %d bytes.\n", bytes_read);
        }

        // Tvinga terminalen att skriva ut direkt
        fflush(stdout);

        // 100ms paus
        usleep(100000);
    }

    close(file);
    return 0;
}
