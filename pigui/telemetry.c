#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "unistd.h"
#include <arpa/inet.h>
#include <sys/socket.h>

#define PORT 5001
#define BUFFER_SIZE 1024

// System states:
// 1 = (auto, auto)
// 2 = (auto, manual)
// 3 = (manual, auto)
// 4 = (manual, manual)

int system_state = 4; // Default to manual/manual

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

int main() {
    int sockfd;
    struct sockaddr_in servaddr, cliaddr;
    unsigned char buffer[BUFFER_SIZE];
    socklen_t len = sizeof(cliaddr);

    // Create UDP socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    // Configure server address
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    // Bind the socket
    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("Bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    printf("Listening for packages on port %d...\n", PORT);
    printf("Initial State: %d %s\n", system_state, get_state_str(system_state));

    // Listening loop
    while (1) {
        int n = recvfrom(sockfd, buffer, BUFFER_SIZE, MSG_WAITALL, (struct sockaddr *)&cliaddr, &len);
        
        if (n < 0) continue;

        // Verification
        // Expected: [0x05, State, Target, Action, 0x00, 0x00, 0x00, 0xFF]
        if (n == 8 && buffer[0] == 0x05 && buffer[7] == 0xFF) {

            unsigned char state = buffer[1];
            unsigned char target = buffer[2];
            char action = (char)buffer[3];

            system_state = state;

            // Print results
            printf("----------------------------------------\n");
            printf("packet recieved from %s:%d\n", inet_ntoa(cliaddr.sin_addr), ntohs(cliaddr.sin_port));
            printf("Run state: %d %s\n", state, get_state_str(state));
            printf("Target: %s\n", get_target_str(target));
            printf("Action: %c\n", action);

            printf("Sensor data: [%02X, %02X, %02X]\n", buffer[4], buffer[5], buffer[6]);
        } else {
            printf("Malformed package. Recieved %d bytes: ", n);
            for (int i = 0; i < n; i++) {
                printf("%02X ", buffer[i]);
            }
            printf("\n");
        }
    }

    close(sockfd);
    return 0;
}   