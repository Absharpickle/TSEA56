#include <avr/io.h>

#define SLAVE_ADDRESS 0x12

volatile uint8_t rx_buffer[8];
volatile uint8_t rx_index = 0;

void TWI_init_slave(void) {
    TWAR = (SLAVE_ADDRESS << 1); 
    TWCR = (1<<TWEN) | (1<<TWEA);
}

void TWI_handle_communication(void) {
    if (!(TWCR & (1<<TWINT))) return;

    uint8_t status = TWSR & 0xF8;

    if (status == 0x60) {
        // Master vill SKRIVA till oss
        rx_index = 0; 
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    } 
    else if (status == 0x80) {
        // Data mottagen
        if (rx_index < 8) {
            rx_buffer[rx_index++] = TWDR; 
        }
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA); 
    } 
    else {
        // STOP eller annat
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    }
}

int main(void) {
    TWI_init_slave();
    while (1) {
        TWI_handle_communication();
    }
    return 0;
}