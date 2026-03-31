#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <unistd.h>    // Krävs för usleep()

// --- MOCKER-SEKTION (Ska vara AV när ni mäter med Digilent) ---
#ifdef SIMULATE_I2C
    // ... (samma mock-kod som tidigare) ...
#else
    #include <fcntl.h>
    #include <sys/ioctl.h>
    #include <linux/i2c-dev.h>
#endif

int send_data_to_slave(int file, int target_address, uint8_t *data, size_t length) {
    if (ioctl(file, 0x0703, target_address) < 0) return -1;
    if (write(file, data, length) != (ssize_t)length) return -1;
    return 0;
}

int main() {
    int file;
    char *filename = "/dev/i2c-1";
    int sensor_addr = 0x11;
    int display_addr = 0x12;

    uint8_t buffer[8] = {0};
    uint8_t reg = 0x00;

    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Kunde inte öppna I2C-bussen");
        return 1;
    }

    printf("Startar kontinuerlig sändning... Tryck Ctrl+C för att avbryta.\n");

    while (1) {
        // --- 1. LÄS FRÅN SENSOR (0x11) ---
        ioctl(file, 0x0703, sensor_addr);
        write(file, &reg, 1); 
        if (read(file, buffer, 8) == 8) {
            printf("Läst från 0x11... ");
        }

        // --- 2. SKICKA TILL DISPLAY (0x12) ---
        if (send_data_to_slave(file, display_addr, buffer, 8) == 0) {
            printf("Skickat till 0x12\n");
        } else {
            printf("Misslyckades skicka till 0x12 (Ingen ACK?)\n");
        }

        // --- 3. PAUSA (100ms) ---
        // Utan denna paus kommer terminalen och bussen bli överfull.
        // 100 000 mikrosekunder = 100 millisekunder.
        usleep(100000); 
    }

    close(file);
    return 0;
}