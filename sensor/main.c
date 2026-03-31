#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>    // För uint8_t, int16_t osv.
#include <sys/types.h>

// --- MOCKER-SEKTION ---
#ifdef SIMULATE_I2C
    #include <stddef.h> 
    #define O_RDWR 2

    int open(const char *path, int flags) { 
        printf("[MOCK] Öppnar virtuell buss: %s\n", path);
        return 99; 
    }
    
    int ioctl(int fd, unsigned long req, ...) { return 0; }
    
    ssize_t write(int fd, const void *buf, size_t count) { return count; }
    
    ssize_t read(int fd, void *buf, size_t count) {
        uint8_t *b = (uint8_t *)buf;
        
        // Vi simulerar ett scenario där vi läser 8 bytes:
        if (count >= 8) {
            // Byte 0: Statusflaggor (Linje #0 = 1, Linje #1 = 0, Hinder = 1) -> 0b00000101 (0x05)
            b[0] = 0x05; 
            
            // Byte 1-2: Linjesensor 0 avvikelse = -15 mm
            // -15 i 16-bitars hex är 0xFFF1. Little-endian: F1 först, sedan FF.
            b[1] = 0xF1; b[2] = 0xFF; 
            
            // Byte 3-4: Linjesensor 1 avvikelse = 20 mm
            // 20 i 16-bitars hex är 0x0014. Little-endian: 14 först, sedan 00.
            b[3] = 0x14; b[4] = 0x00; 
            
            // Byte 5: Hinderavstånd = 35 cm
            b[5] = 35;   
            
            // Byte 6-7: Gyro vinkelhastighet = -25.5 grader/s
            // -25.5 * 10 = -255. I 16-bit hex är det 0xFF01. Little-endian: 01 först, sedan FF.
            b[6] = 0x01; b[7] = 0xFF; 
            
            printf("[MOCK] Skickar 8 bytes sensordata...\n\n");
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

int main() {
    int file;
    char *filename = "/dev/i2c-1";
    int addr = 0x11;

    unsigned char reg = 0x00;
    
    // Vi skapar en buffer som rymmer alla 8 bytes
    uint8_t buffer[8] = {0};

    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open bus");
        return 1;
    }

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

    // Vi begär 8 bytes från slaven
    if (read(file, buffer, 8) != 8) {
        perror("Failed to read all 8 bytes from device");
        close(file);
        return 1;
    }

    // --- TOLKA DATAN ---
    printf("=== SENSORDATA MOTTAGEN ===\n");

    // Byte 0: Statusflaggor (Bitmaskning)
    uint8_t status = buffer[0];
    uint8_t line0_detected = (status & (1 << 0)) != 0;
    uint8_t line1_detected = (status & (1 << 1)) != 0;
    uint8_t obstacle_detected = (status & (1 << 2)) != 0;
    
    printf("Status:\n");
    printf("  - Linje 0: %s\n", line0_detected ? "Ja" : "Nej");
    printf("  - Linje 1: %s\n", line1_detected ? "Ja" : "Nej");
    printf("  - Hinder:  %s\n", obstacle_detected ? "Ja" : "Nej");

    // Byte 1-2: Linjesensor 0 (int16_t, little-endian)
    // Pussla ihop: (Mest signifikant byte shiftad 8 steg) ELLER (Minst signifikant byte)
    int16_t dev0 = (int16_t)(buffer[1] | (buffer[2] << 8));
    printf("Avvikelse linje 0: %d mm\n", dev0);

    // Byte 3-4: Linjesensor 1 (int16_t, little-endian)
    int16_t dev1 = (int16_t)(buffer[3] | (buffer[4] << 8));
    printf("Avvikelse linje 1: %d mm\n", dev1);

    // Byte 5: Hinderavstånd (uint8_t)
    uint8_t distance = buffer[5];
    printf("Avstånd till hinder: %d cm\n", distance);

    // Byte 6-7: Gyro (int16_t, little-endian, upplösning 0.1 grader/sekund)
    int16_t gyro_raw = (int16_t)(buffer[6] | (buffer[7] << 8));
    float gyro_deg_s = gyro_raw / 10.0f;
    printf("Gyro vinkelhastighet: %.1f grader/s\n", gyro_deg_s);
    
    printf("===========================\n");

    close(file);
    return 0;
}