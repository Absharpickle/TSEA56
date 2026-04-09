#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

int main() {
    int file;
    char *filename = "/dev/i2c-1";
    int avr_addr = 0x12; // AVR:ens adress

    uint8_t buffer[8];

    // 1. Öppna bussen
    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Kunde inte öppna I2C-bussen");
        return 1;
    }

    // 2. Anslut till AVR:en
    if (ioctl(file, I2C_SLAVE, avr_addr) < 0) {
        perror("Kunde inte ansluta till adressen");
        close(file);
        return 1;
    }

    printf("Skickar 8 bytes kontinuerligt till 0x12...\n");

    while (1) {
        // Fyll bufferten med testdata (samma som tidigare)
        buffer[0] = 0x05; 
        buffer[1] = 0xF1; buffer[2] = 0xFF; 
        buffer[3] = 0x14; buffer[4] = 0x00; 
        buffer[5] = 35;   
        buffer[6] = 0x01; buffer[7] = 0xFF; 

        // Skicka iväg datan
        if (write(file, buffer, 8) == 8) {
            printf("Skickade 8 bytes (ACK mottaget)!\n");
        } else {
            printf("Fel vid sändning (Ingen AVR ansluten / NACK).\n");
        }

        usleep(250000); // Pausa 250ms
    }

    close(file);
    return 0;
}