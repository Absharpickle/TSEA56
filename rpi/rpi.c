#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

int main() {
    int file;
    char *filename = "/dev/i2c-1";
    int addr = XXX;

    unsigned char reg = 0x00;
    unsigned char buffer[1] = {0};

    // Open the I2C bus
    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open bus");
        return 1;
    }

    // Connect to device
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        perror("Failed to connect to device");
        close(file);
        return 1;
    }

    // Write register address to read from
    if (write(file, &reg, 1) != 1) {
        perror("Failed to write register address to device");
        close(file);
        return 1;
    }

    // Read data from device
    if (read(file, buffer, 1) != 1) {
        perror("Failed to read data from device");
        close(file);
        return 1;
    }

    // Print result in hex format
    printf("Data received: 0x%02X\n", buffer[0]);

    close(file);
    return 0;
}