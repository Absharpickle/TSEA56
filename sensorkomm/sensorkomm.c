#include <avr/io.h>
#include <avr/interrupt.h>

#define SLAVE_ADDRESS 0x11
volatile uint8_t tx_buffer[8] = {0x05, 0xF1, 0xFF, 0x14, 0x00, 35, 0x01, 0xFF};
volatile uint8_t tx_index = 0;

void TWI_init_slave(void) {
    TWAR = (SLAVE_ADDRESS << 1); 
    TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE); 
    sei(); 
}

ISR(TWI_vect) {
    uint8_t status = TWSR & 0xF8;

    if (status == 0xA8) { // Master vill LÄSA
        tx_index = 0;
        TWDR = tx_buffer[tx_index++];
    } else if (status == 0xB8) { // Master vill ha NÄSTA byte
        TWDR = (tx_index < 8) ? tx_buffer[tx_index++] : 0x00;
    } 
    // ALLA andra tillstånd (inklusive 0xC0 NACK och fel) ska nollställa flaggan:
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
}

int main(void) {
    TWI_init_slave();
    while (1); 
}