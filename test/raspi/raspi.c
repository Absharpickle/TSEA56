#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <string.h>

int main() {
    int file;
    const char *device = "/dev/i2c-1";
    int addr = 0x11;
    uint8_t buffer[8];

    printf("Öppnar I2C-enhet...\n");
    if ((file = open(device, O_RDWR)) < 0) {
        perror("FEL: Kunde inte öppna bussen");
        return 1;
    }

    printf("Ansluter till slav 0x%02X...\n", addr);
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        perror("FEL: Kunde inte sätta adress");
        close(file);
        return 1;
    }

    printf("Loggning startad. Väntar på data (Tryck Ctrl+C för att avbryta)...\n");

    while (1) {
        // Vi skriver ut detta för att se om loopen snurrar
        printf("Försöker läsa... ");
        fflush(stdout); 

        int bytes_read = read(file, buffer, 8);
        
        if (bytes_read == 8) {
            printf("Lyckades! Sparar till fil.\n");
            
            FILE *f = fopen("sensor_log.txt", "a");
            if (f) {
                fprintf(f, "Data: ");
                for(int i=0; i<8; i++) fprintf(f, "%02X ", buffer[i]);
                fprintf(f, "\n");
                fclose(f);
            }
        } else {
            printf("Misslyckades. Läste %d bytes. Error: %s\n", bytes_read, strerror(errno));
        }

        usleep(500000); // Öka till 500ms för att inte spamma terminalen vid fel
    }

    close(file);
    return 0;
}