#include <stdio.h>
#include <stdlib.h>
#include <libusb-1.0/libusb.h>
#include <unistd.h>

#define VENDOR_ID 0x24c6
#define PRODUCT_ID 0x542a
#define ENDPOINT_IN 0x81 

int main() {
    libusb_context *ctx = NULL;
    libusb_device_handle *dev_handle = NULL;
    int res, transferred;
    unsigned char buf[64];

    if (libusb_init(&ctx) < 0) return 1;

    dev_handle = libusb_open_device_with_vid_pid(ctx, VENDOR_ID, PRODUCT_ID);
    if (!dev_handle) {
        printf("Kunde inte öppna kontrollen.\n");
        return 1;
    }
    
    if (libusb_claim_interface(dev_handle, 0) < 0) return 1;

    printf("Gränssnitt övertaget. Skickar det STORA Linux-handslaget...\n");

    // --- LINUX KERNEL XBOX ONE WAKE-UP SEQUENCE ---
    // Paket 1: Ström på
    unsigned char magic_1[] = {0x05, 0x20, 0x00, 0x01, 0x00};
    // Paket 2: Bekräftelse och protokoll-synk
    unsigned char magic_2[] = {0x0A, 0x20, 0x00, 0x03, 0x00, 0x01, 0x14};
    // Paket 3: Aktivera joystick-data och Force Feedback (Rumble)
    unsigned char magic_3[] = {0x09, 0x00, 0x00, 0x09, 0x00, 0x0F, 0x00, 0x00, 0x1D, 0x1D, 0xFF, 0x00, 0x00};

    // Vi vet inte säkert om din kontroll lyssnar på kanal 1 eller 2, så vi bombarderar båda!
    for (int ep = 1; ep <= 2; ep++) {
        libusb_interrupt_transfer(dev_handle, ep, magic_1, sizeof(magic_1), &transferred, 50);
        libusb_interrupt_transfer(dev_handle, ep, magic_2, sizeof(magic_2), &transferred, 50);
        libusb_interrupt_transfer(dev_handle, ep, magic_3, sizeof(magic_3), &transferred, 50);
    }

    printf("Handslag skickat. Väntar på speldata...\n");
    usleep(500000); // Ge kontrollen en halv sekund att vakna

    while (1) {
        res = libusb_interrupt_transfer(dev_handle, ENDPOINT_IN, buf, sizeof(buf), &transferred, 100);

        if (res == 0 && transferred > 0) {
            // Om paketet börjar på 0x20 (Speldata) och har rätt längd, visa det!
            if (buf[0] == 0x20) {
                printf("\nSPELDATA (%02d bytes): ", transferred);
                for (int i = 0; i < transferred; i++) {
                    printf("%02x ", buf[i]);
                }
            } 
            // Om vi fortfarande bara får det gamla announce-paketet (0x02) skriver vi ett 'A'
            else if (buf[0] == 0x02) {
                printf("A ");
                fflush(stdout);
            }
        } 
        else if (res == LIBUSB_ERROR_TIMEOUT) {
            printf(".");
            fflush(stdout); 
        } 
        else {
            break;
        }
    }

    libusb_release_interface(dev_handle, 0);
    libusb_close(dev_handle);
    libusb_exit(ctx);
    return 0;
}