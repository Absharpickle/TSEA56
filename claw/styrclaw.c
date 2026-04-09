#define F_CPU 8000000UL 
#include <avr/io.h>
#include <avr/interrupt.h>

#define SLAVE_ADDRESS 0x12

volatile uint8_t rx_buffer[8];
volatile uint8_t tx_buffer[8] = {0x00};
volatile uint8_t rx_index = 0;
volatile uint8_t tx_index = 0;
volatile uint8_t nytt_paket = 0;

// Fiktiva variabler för robotens tillstånd
int8_t pos_x = 10;
int8_t pos_y = 10;
int8_t vippa_vinkel = 45;
int8_t rotations_vinkel = 0;
uint8_t klo_stangd = 0; // 0 = Öppen, 1 = Stängd

void TWI_init_slave(void) {
    TWAR = (SLAVE_ADDRESS << 1); 
    TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
    sei(); 
}

ISR(TWI_vect) {
    uint8_t status = TWSR & 0xF8;

    // --- Pi:n SKRIVER ---
    if (status == 0x60) { 
        rx_index = 0;
    } else if (status == 0x80) { 
        if (rx_index < 8) rx_buffer[rx_index++] = TWDR;
    } else if (status == 0xA0) { 
        nytt_paket = 1; 
    }
    
    // --- Pi:n LÄSER (Status-begäran) ---
    else if (status == 0xA8) { 
        tx_index = 0;
        TWDR = tx_buffer[tx_index++];
    } else if (status == 0xB8) { 
        TWDR = (tx_index < 8) ? tx_buffer[tx_index++] : 0xFF;
    }

    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
}

int main(void) {
    TWI_init_slave();

    while (1) {
        if (nytt_paket) {
            if (rx_buffer[0] == 0x05 && rx_buffer[7] == 0xFF) {
                
                uint8_t lage = rx_buffer[1];
                int8_t varde = (int8_t)rx_buffer[2];

                // --- Uppdatera interna variabler baserat på tangenttryck ---
                if (lage == 1) { // X/Y
                    if (varde == 1) pos_y++;
                    else if (varde == -1) pos_y--;
                    else if (varde == 2) pos_x++;
                    else if (varde == -2) pos_x--;
                } 
                else if (lage == 2) { // Vippa
                    if (varde == 1) vippa_vinkel += 5;
                    else if (varde == -1) vippa_vinkel -= 5;
                } 
                else if (lage == 3) { // Rotation
                    if (varde == 1) rotations_vinkel += 10;
                    else if (varde == -1) rotations_vinkel -= 10;
                } 
                else if (lage == 4) { // Klo
                    if (varde == 1) klo_stangd = 1;
                    else if (varde == 0) klo_stangd = 0;
                }
                
                // --- Läge 5: Pi:n vill veta hur vi står just nu! ---
                else if (lage == 5) { 
                    tx_buffer[0] = 0x05; // Start
                    tx_buffer[1] = (uint8_t)pos_x;
                    tx_buffer[2] = (uint8_t)pos_y;
                    tx_buffer[3] = (uint8_t)vippa_vinkel;
                    tx_buffer[4] = (uint8_t)rotations_vinkel;
                    tx_buffer[5] = klo_stangd;
                    tx_buffer[6] = 0x00;
                    tx_buffer[7] = 0xFF; // Stopp
                }
            }
            nytt_paket = 0; 
        }
    }
}