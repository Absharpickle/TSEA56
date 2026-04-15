#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>

/*
 * Förväntat paketformat från Sensormodulen (8 bytes):
 * [0]: status (bitmask)
 * [1-2]: dev0 (int16_t)
 * [3-4]: dev1 (int16_t)
 * [5]: dist (uint8_t)
 * [6-7]: omega (int16_t)
 */

void save_to_file(uint8_t* buf) {
    FILE *f = fopen("sensor_log.txt", "a");
    if (f == NULL) return;

    // Tidstämpel för loggen
    time_t rawtime;
    struct tm *timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    char time_str[20];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", timeinfo);

    // Tolka bytes (Little Endian)
    int16_t dev0  = (int16_t)(buf[1] | (buf[2] << 8));
    int16_t dev1  = (int16_t)(buf[3] | (buf[4] << 8));
    uint8_t dist  = buf[5];
    int16_t omega = (int16_t)(buf[6] | (buf[7] << 8));
    uint8_t stat  = buf[0];

    // Skriv till filen
    fprintf(f, "[%s] STAT: 0x%02X | Dev0: %6d | Dev1: %6d | Dist: %3u | Gyro: %6d\n",
            time_str, stat, dev0, dev1, dist, omega);

    fclose(f);
}

int main() {
    int file;
    const char *device = "/dev/i2c-1";
    const int addr = 0x11; // Sensorns adress
    uint8_t buffer[8];

    // 1. Initiera I2C-bussen
    if ((file = open(device, O_RDWR)) < 0) {
        perror("Kunde inte öppna I2C-bussen");
        return 1;
    }

    // 2. Sätt slav-adress
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        perror("Kunde inte hitta sensorn");
        close(file);
        return 1;
    }

    printf("Loggning startad. Läser från 0x%02X varje 100ms...\n", addr);

    while (1) {
        // 3. Läs 8 bytes från sensorn
        if (read(file, buffer, 8) == 8) {
            save_to_file(buffer);
            printf("Data sparad i sensor_log.txt\n");
        } else {
            printf("Fel vid läsning från sensor.\n");
        }

        // Pausa 100ms (100 000 mikrosekunder)
        usleep(100000);
    }

    close(file);
    return 0;
}