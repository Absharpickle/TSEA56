#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>

// --- MOCKER-SEKTION (Behålls för testning, men stängs av för fysiska pinnar) ---
#ifdef SIMULATE_I2C
    #include <stddef.h> 
    #define O_RDWR 2
    int open(const char *path, int flags) { return 99; }
    int ioctl(int fd, unsigned long req, int addr) { 
        printf("[MOCK] Bytte fokus till I2C-adress: 0x%02X\n", addr);
        return 0; 
    }
    ssize_t write(int fd, const void *buf, size_t count) { 
        printf("[MOCK] Skriver %zu bytes...\n", count);
        return count; 
    }
    ssize_t read(int fd, void *buf, size_t count) {
        if (count >= 8) {
            uint8_t *b = (uint8_t *)buf;
            b[0]=0x05; b[1]=0xF1; b[2]=0xFF; b[3]=0x14; b[4]=0x00; b[5]=35; b[6]=0x01; b[7]=0xFF;
        }
        return count;
    }
    int close(int fd) { return 0; }
#else
    #include <fcntl.h>
    #include <unistd.h>
    #include <sys/ioctl.h>
    #include <linux/i2c-dev.h>
#endif


// --- NY FUNKTION: Skicka data till en specifik adress ---
int send_data_to_slave(int file, int target_address, uint8_t *data, size_t length) {
    // 1. Byt mål-adress på I2C-bussen
    if (ioctl(file, 0x0703, target_address) < 0) { // 0x0703 är I2C_SLAVE
        perror("Kunde inte byta till ny slavadress");
        return -1;
    }

    // 2. Skicka datan
    if (write(file, data, length) != length) {
        perror("Misslyckades att skriva data till slaven");
        return -1;
    }

    printf("Framgång: Skickade %zu bytes till adress 0x%02X\n", length, target_address);
    return 0;
}


int main() {
    int file;
    char *filename = "/dev/i2c-1";
    int sensor_addr = 0x11;  // Slaven vi läser ifrån
    int display_addr = 0x12; // NY SLAV: Den vi skickar till (decimalt 18)

    uint8_t sensor_buffer[8] = {0};

    // Öppna bussen
    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open bus");
        return 1;
    }

    // --- DEL 1: LÄS FRÅN SENSOR (0x11) ---
    if (ioctl(file, 0x0703, sensor_addr) < 0) {
        perror("Failed to connect to sensor");
        close(file); return 1;
    }
    
    unsigned char reg = 0x00;
    write(file, &reg, 1); // Be om att få läsa från register 0
    
    if (read(file, sensor_buffer, 8) != 8) {
        perror("Failed to read data");
        close(file); return 1;
    }
    printf("Data inläst från sensor (0x%02X)!\n", sensor_addr);


    // --- DEL 2: SKICKA DATA TILL NY SLAV (0x12) ---
    // Låt oss säga att vi vill skicka vidare exakt samma 8 bytes vi just tog emot
    // till den nya slaven. Vi kan också bygga en ny array om vi vill omformatera datan.
    
    printf("\nFörsöker skicka data vidare...\n");
    send_data_to_slave(file, display_addr, sensor_buffer, 8);


    close(file);
    return 0;
}