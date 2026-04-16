#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <time.h>

#define UDP_PORT 5001
#define BUFFER_SIZE 1024

// I2C Configuration
#define I2C_DEVICE "/dev/i2c-1"
#define STYRKOMM_ADDR 0x12
#define PACKET_SIZE 8
#define VERIFY_LOG_FILE "verifikation_keys.txt"

// Helper functions for readable output
const char* get_state_str(int state) {
    switch (state) {
        case 1: return "(auto, auto)";
        case 2: return "(auto, manual)";
        case 3: return "(manual, auto)";
        case 4: return "(manual, manual)";
        default: return "Unknown state";
    }
}

const char* get_target_str(int target) {
    if (target == 0) return "wheel";
    if (target == 1) return "arm";
    return "Unknown target";
}

// Function to log the forwarded packets
void log_verification(const unsigned char *sent, char action) {
    FILE *f = fopen(VERIFY_LOG_FILE, "a");
    if (f == NULL) return;

    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    fprintf(f, "[%02d:%02d:%02d] ACTION '%c'\n",
            t->tm_hour, t->tm_min, t->tm_sec, action);

    fprintf(f, "SKICKAT (0x12): ");
    for (int i = 0; i < PACKET_SIZE; i++) {
        fprintf(f, "%02X ", sent[i]);
    }
    fprintf(f, "\n\n");

    fclose(f);
}

int main() {
    int sockfd, i2c_fd;
    struct sockaddr_in servaddr, cliaddr;
    unsigned char buffer[BUFFER_SIZE];
    socklen_t len = sizeof(cliaddr);

    // 1. CLEAR OLD LOG FILE
    FILE *clr = fopen(VERIFY_LOG_FILE, "w");
    if (clr) fclose(clr);
    printf("--- PI TELEMETRY BRIDGE: UDP -> I2C ---\n");
    printf("Verifikation loggas till: %s\n\n", VERIFY_LOG_FILE);

    // 2. SETUP I2C
    i2c_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd < 0) {
        perror("Failed to open I2C bus (/dev/i2c-1)");
        exit(EXIT_FAILURE);
    }

    if (ioctl(i2c_fd, I2C_SLAVE, STYRKOMM_ADDR) < 0) {
        perror("Failed to connect to I2C address 0x12");
        close(i2c_fd);
        exit(EXIT_FAILURE);
    }
    printf("Successfully connected to I2C address 0x12\n");

    // 3. SETUP UDP SOCKET
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        close(i2c_fd);
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(UDP_PORT);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("Bind failed");
        close(sockfd);
        close(i2c_fd);
        exit(EXIT_FAILURE);
    }
    printf("Listening for UDP packets on port %d...\n\n", UDP_PORT);

    // 4. MAIN LISTENING LOOP
    while (1) {
        int n = recvfrom(sockfd, buffer, BUFFER_SIZE, MSG_WAITALL, (struct sockaddr *)&cliaddr, &len);
        
        if (n < 0) continue;

        // Protocol Verification: [0x05, State, Target, Action, 0x00, 0x00, 0x00, 0xFF]
        if (n == PACKET_SIZE && buffer[0] == 0x05 && buffer[7] == 0xFF) {

            unsigned char state = buffer[1];
            unsigned char target = buffer[2];
            char action = (char)buffer[3];

            // Print to console
            printf("----------------------------------------\n");
            printf("Packet recieved from %s:%d\n", inet_ntoa(cliaddr.sin_addr), ntohs(cliaddr.sin_port));
            printf("Run state: %d %s\n", state, get_state_str(state));
            printf("Target: %s\n", get_target_str(target));
            printf("Action: %c (Hex: 0x%02X)\n", action, buffer[3]);

            // --- FORWARD TO I2C ---
            ssize_t written = write(i2c_fd, buffer, PACKET_SIZE);
            if (written != PACKET_SIZE) {
                fprintf(stderr, "I2C Write Failed (errno=%d: %s)\n", errno, strerror(errno));
            } else {
                printf("=> Successfully forwarded to TWI 0x12\n");
                log_verification(buffer, action);
            }

        } else {
            printf("Malformed packet. Received %d bytes: ", n);
            for (int i = 0; i < n; i++) printf("%02X ", buffer[i]);
            printf("\n");
        }
    }

    close(sockfd);
    close(i2c_fd);
    return 0;
}