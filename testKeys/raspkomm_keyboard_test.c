// filepath: x:/Documents/robot/TSEA56/raspkomm/raspkomm_keyboard_test.c
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#define I2C_DEVICE "/dev/i2c-1"
#define STYRKOMM_ADDR 0x12
#define PACKET_SIZE 8

static struct termios g_orig_termios;
static int g_i2c_fd = -1;

static void restore_terminal(void) {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
}

static void cleanup_and_exit(int code) {
    restore_terminal();
    if (g_i2c_fd >= 0) {
        close(g_i2c_fd);
        g_i2c_fd = -1;
    }
    exit(code);
}

static void signal_handler(int sig) {
    (void)sig;
    cleanup_and_exit(0);
}

static bool enable_raw_terminal(void) {
    if (tcgetattr(STDIN_FILENO, &g_orig_termios) < 0) {
        perror("tcgetattr");
        return false;
    }

    struct termios raw = g_orig_termios;
    raw.c_lflag &= (tcflag_t) ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) < 0) {
        perror("tcsetattr");
        return false;
    }

    return true;
}

static bool send_command_to_styrkomm(char cmd) {
    uint8_t buffer[PACKET_SIZE] = {0};
    buffer[0] = 0x05;             // Paket-ID
    buffer[1] = 1;                // Fall 1: direktkommando
    buffer[2] = (uint8_t)cmd;     // Kommando: f/l/r/b
    buffer[3] = 0x00;
    buffer[4] = 0x00;
    buffer[5] = 0x00;
    buffer[6] = 0x00;
    buffer[7] = 0xFF;             // Slutbyte

    if (ioctl(g_i2c_fd, I2C_SLAVE, STYRKOMM_ADDR) < 0) {
        perror("ioctl(I2C_SLAVE)");
        return false;
    }

    ssize_t written = write(g_i2c_fd, buffer, PACKET_SIZE);
    if (written != PACKET_SIZE) {
        perror("write");
        return false;
    }

    return true;
}

static int read_key(void) {
    unsigned char c = 0;
    ssize_t n = read(STDIN_FILENO, &c, 1);
    if (n <= 0) return -1;

    if (c == 'q' || c == 'Q') return 'q';

    if (c == 0x1B) { // ESC-sekvens för piltangenter
        unsigned char seq[2] = {0};
        if (read(STDIN_FILENO, &seq[0], 1) <= 0) return -1;
        if (read(STDIN_FILENO, &seq[1], 1) <= 0) return -1;

        if (seq[0] == '[') {
            return seq[1]; // 'A','B','C','D'
        }
    }

    return -1;
}

int main(void) {
    printf("--- KEYBOARD TEST: RASPBERRY PI -> STYRKOMM ---\n");
    printf("Piltangent Upp    = fram ('f')\n");
    printf("Piltangent Vänster= vänster ('l')\n");
    printf("Piltangent Höger  = höger ('r')\n");
    printf("Piltangent Ner    = bakåt ('b')\n");
    printf("Tryck 'q' för att avsluta.\n\n");

    g_i2c_fd = open(I2C_DEVICE, O_RDWR);
    if (g_i2c_fd < 0) {
        perror("open(/dev/i2c-1)");
        return 1;
    }

    if (!enable_raw_terminal()) {
        close(g_i2c_fd);
        return 1;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    while (true) {
        int key = read_key();
        char cmd = '\0';

        switch (key) {
            case 'A': cmd = 'f'; break; // Upp
            case 'B': cmd = 'b'; break; // Ner
            case 'C': cmd = 'r'; break; // Höger
            case 'D': cmd = 'l'; break; // Vänster
            case 'q':
                printf("\nAvslutar...\n");
                cleanup_and_exit(0);
                break;
            default:
                break;
        }

        if (cmd != '\0') {
            if (send_command_to_styrkomm(cmd)) {
                printf("Skickade kommando: '%c'\n", cmd);
            } else {
                fprintf(stderr, "Misslyckades skicka kommando '%c' (errno=%d: %s)\n",
                        cmd, errno, strerror(errno));
            }
        }
    }

    cleanup_and_exit(0);
    return 0;
}