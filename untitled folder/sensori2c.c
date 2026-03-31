#include <stdio.h>

// Definiera denna för att köra i "Simuleringsläge"
#define SIMULATE_I2C 

#ifdef SIMULATE_I2C
    // Vi "skuggar" de riktiga funktionerna
    int open(const char *path, int flags) { return 99; } // Returnera ett fejk-id
    int ioctl(int fd, unsigned long req, ...) { return 0; }
    int close(int fd) { return 0; }

    ssize_t write(int fd, const void *buf, size_t count) {
        printf("[SIM] Master skrev register: 0x%02X\n", ((unsigned char*)buf)[0]);
        return count;
    }

    ssize_t read(int fd, void *buf, size_t count) {
        ((unsigned char*)buf)[0] = 0xAA; // Här skickar vi tillbaka vårt "ATmega-svar"
        printf("[SIM] Master läste värde: 0x%02X\n", ((unsigned char*)buf)[0]);
        return count;
    }
#else
    #include <fcntl.h>
    #include <unistd.h>
    #include <sys/ioctl.h>
    #include <linux/i2c-dev.h>
#endif

int main() {
    // Din kod här... (den kommer nu använda fusk-funktionerna ovan)
    // ... samma kod som du skrev tidigare ...
    printf("Data received: 0x%02X\n", buffer[0]);
}