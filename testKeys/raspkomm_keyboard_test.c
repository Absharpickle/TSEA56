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
#include <time.h>

#define I2C_DEVICE "/dev/i2c-1"
#define STYRKOMM_ADDR 0x12
#define PACKET_SIZE 8
#define VERIFY_LOG_FILE "verifikation_keys.txt"


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

static void log_verification(const uint8_t *sent, const uint8_t *received, ssize_t received_len, char cmd) {
    FILE *f = fopen(VERIFY_LOG_FILE, "a");
    if (f == NULL) return;

    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    fprintf(f, "[%02d:%02d:%02d] CMD '%c'\n",
            t->tm_hour, t->tm_min, t->tm_sec, cmd);

    fprintf(f, "SKICKAT (0x12): ");
    for (int i = 0; i < PACKET_SIZE; i++) fprintf(f, "%02X ", sent[i]);

    fprintf(f, "\nECHO    (0x12): ");
    if (received_len == PACKET_SIZE) {
        for (int i = 0; i < PACKET_SIZE; i++) fprintf(f, "%02X ", received[i]);
        fprintf(f, " | %s\n\n", (memcmp(sent, received, PACKET_SIZE) == 0) ? "MATCH" : "FEL");
    } else {
        fprintf(f, "<read_len=%zd> | FEL\n\n", received_len);
    }

    fclose(f);
}

static bool send_command_to_styrkomm(char cmd) {
    uint8_t sent[PACKET_SIZE] = {0};
    uint8_t echo[PACKET_SIZE] = {0};
    sent[0] = 0x05;
    sent[1] = (unint8_t)state;
    sent[2] = (uint8_t)cmd;
    sent[7] = 0xFF;

    if (ioctl(g_i2c_fd, I2C_SLAVE, STYRKOMM_ADDR) < 0) {
        perror("ioctl(I2C_SLAVE)");
        return false;
    }

    ssize_t written = write(g_i2c_fd, sent, PACKET_SIZE);
    if (written != PACKET_SIZE) {
        perror("write");
        return false;
    }

    usleep(5000);
    ssize_t r = read(g_i2c_fd, echo, PACKET_SIZE);
    log_verification(sent, echo, r, cmd);

    return (r == PACKET_SIZE) && (memcmp(sent, echo, PACKET_SIZE) == 0);
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
    printf("Stopp ('s')\n");
    printf("Medsols ('e')\n");
    printf("Motsols ('o')\n");
    printf("Plocka ('v')\n");
    FILE *clr = fopen(VERIFY_LOG_FILE, "w"); // clear old log each run
    if (clr) fclose(clr);
    printf("Verifikation loggas till: %s\n\n", VERIFY_LOG_FILE);
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
            case 'A': cmd = 'f'; state = 1; break; // Upp
            case 'B': cmd = 'b'; state = 1; break; // Ner
            case 'C': cmd = 'r'; state = 1; break; // Höger
            case 'D': cmd = 'l'; state = 1; break; // Vänster
            case 's': cmd = 's'; state = 1; break; // Stopp
            case 'e': cmd = 'e'; state = 4; break; // Medsols
            case 'o': cmd = 'o'; state = 4; break; // Motsols
            case 'v': cmd = 'v'; state = 4; break; // Plocka
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