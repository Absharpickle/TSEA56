#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h> // Innehåller definitionen av ssize_t för hela systemet

// --- MOCKER-SEKTION ---
#ifdef SIMULATE_I2C
    #include <stddef.h> 
    #define O_RDWR 2

    // Vi tar bort "typedef int ssize_t" eftersom den finns i sys/types.h
    
    int open(const char *path, int flags) { 
        printf("[MOCK] Öppnar virtuell buss: %s\n", path);
        return 99; 
    }
    
    // Vi använder "unsigned long" för req för att matcha systemets ioctl-signatur
    int ioctl(int fd, unsigned long req, ...) { 
        printf("[MOCK] I2C_SLAVE operation utförd.\n");
        return 0; 
    }
    
    ssize_t write(int fd, const void *buf, size_t count) {
        printf("[MOCK] Skriver register: 0x%02X\n", *(unsigned char*)buf);
        return count;
    }
    
    ssize_t read(int fd, void *buf, size_t count) {
        ((unsigned char*)buf)[0] = 0xAA; 
        printf("[MOCK] Läser data... Svarar: 0xAA\n");
        return count;
    }
    
    int close(int fd) { return 0; }
#else
    #include <fcntl.h>
    #include <unistd.h>
    #include <sys/ioctl.h>
    #include <linux/i2c-dev.h>
#endif

int main() {
    int file;
    char *filename = "/dev/i2c-1";
    int addr = 0x11;

    unsigned char reg = 0x00;
    unsigned char buffer[1] = {0};

    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open bus");
        return 1;
    }

    // Vi använder 0x0703 (I2C_SLAVE) direkt för att slippa inkludera linux-headers i mock-läge
    if (ioctl(file, 0x0703, addr) < 0) {
        perror("Failed to connect to device");
        close(file);
        return 1;
    }

    if (write(file, &reg, 1) != 1) {
        perror("Failed to write register");
        close(file);
        return 1;
    }

    if (read(file, buffer, 1) != 1) {
        perror("Failed to read data");
        close(file);
        return 1;
    }

    printf("RESULTAT: Data mottagen: 0x%02X\n", buffer[0]);

    close(file);
    return 0;
}