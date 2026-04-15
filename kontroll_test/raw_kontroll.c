#include <stdio.h>
#include <stdlib.h>
#include <hidapi/hidapi.h>
#include <unistd.h>

#define VENDOR_ID 0x24c6
#define PRODUCT_ID 0x542a

int main() {
    int res;
    unsigned char buf[64]; // En buffer för att fånga upp USB-paketen
    hid_device *handle;

    // 1. Initiera HIDAPI
    if (hid_init() != 0) {
        printf("Fel: Kunde inte initiera hidapi.\n");
        return 1;
    }

    printf("Letar efter PowerA Spectra (VID: 0x%04x, PID: 0x%04x)...\n", VENDOR_ID, PRODUCT_ID);

    // 2. Öppna enheten direkt via USB
    handle = hid_open(VENDOR_ID, PRODUCT_ID, NULL);

    if (!handle) {
        printf("Kunde inte öppna handkontrollen.\n");
        printf("TIPS: Macen blockerar ibland åtkomst. Testa att köra programmet med 'sudo'.\n");
        hid_exit();
        return 1;
    }

    printf("SUCCÉ! Handkontroll öppnad på raw USB-nivå.\n");
    printf("Tryck på knappar och spakar för att se datan ändras.\n");
    printf("(Tryck Ctrl+C för att avsluta)\n\n");

    // 3. Huvudloop: Läs data kontinuerligt
    while (1) {
        // Läs max 64 bytes med en timeout på 50 millisekunder
        res = hid_read_timeout(handle, buf, sizeof(buf), 50);

        if (res < 0) {
            printf("Fel vid läsning från USB.\n");
            break;
        } else if (res > 0) {
            // Vi fick ett datapaket! Printa ut det i hexadecimalt format
            printf("Mottog %02d bytes: ", res);
            
            for (int i = 0; i < res; i++) {
                // Printa varje byte (00 till FF)
                printf("%02x ", buf[i]);
            }
            printf("\n");
        }
        
        // Liten paus för processorn
        usleep(1000 * 10); // 10 millisekunder
    }

    // 4. Städa upp
    hid_close(handle);
    hid_exit();

    return 0;
}