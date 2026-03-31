#include <stdio.h>
#include <stdlib.h>

// --- MOCKER-SEKTION ---
// Om vi kompilerar med -DSIMULATE_I2C så körs dessa istället för de riktiga
#ifdef SIMULATE_I2C
    #define O_RDWR 2
    typedef int ssize_t;
    int open(const char *path, int flags) { 
        printf("[MOCK] Öppnar virtuell buss: %s\n", path);
        return 99; 
    }
    int ioctl(int fd, unsigned long req, int addr) { 
        printf("[MOCK] Ansluter till adress: 0x%02X\n", addr);
        return 0; 
    }
    ssize_t write(int fd, const void *buf, size_t count) {
        printf("[MOCK] Skriver register: 0x%02X\n", *(unsigned char*)buf);
        return count;
    }
    ssize_t read(int fd, void *buf, size_t count) {
        ((unsigned char*)buf)[0] = 0xAA; // Här bestämmer vi vad "ATmegan" svarar
        printf("[MOCK] Läser data från enhet... Svarar: 0xAA\n");
        return count;
    }
    int close(int fd) { return 0; }
#else
    // Om vi inte simulerar, använd vanliga systembibliotek
    #include <fcntl.h>
    #include <unistd.h>
    #include <sys/ioctl.h>
    #include <linux/i2c-dev.h>
#endif

int main() {
    int file;
    char *filename = "/dev/i2c-1";
    int addr = 0x11; // Din ATmega-adress

    unsigned char reg = 0x00;
    unsigned char buffer[1] = {0};

    // 1. Öppna bussen
    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open bus");
        return 1;
    }

    // 2. Anslut till enhet
    if (ioctl(file, 0x0703, addr) < 0) { // 0x0703 är I2C_SLAVE
        perror("Failed to connect to device");
        close(file);
        return 1;
    }

    // 3. Skriv registeradress (t.ex. 0x00)
    if (write(file, &reg, 1) != 1) {
        perror("Failed to write register");
        close(file);
        return 1;
    }

    // 4. Läs data
    if (read(file, buffer, 1) != 1) {
        perror("Failed to read data");
        close(file);
        return 1;
    }

    printf("RESULTAT: Data mottagen: 0x%02X (binärt: 10101010)\n", buffer[0]);

    close(file);
    return 0;
}