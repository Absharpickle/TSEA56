#include <avr/io.h>
#include <avr/interrupt.h>

#define SLAVE_ADDRESS 0x11

void TWI_init_slave(void) {
    // 1. Sätt slavens adress i TWAR-registret
    TWAR = (SLAVE_ADDRESS << 1); 
    
    // 2. Konfigurera kontrollregistret (TWCR)
    // TWEN: Aktivera TWI
    // TWEA: Enable Acknowledge (svara med ACK på vår adress)
    TWCR = (1<<TWEN) | (1<<TWEA);
}

// Enkel funktion för att skicka en byte när Master efterfrågar det
void TWI_transmit_byte(uint8_t data) {
    // Vänta på att TWINT sätts (Master vill läsa)
    while (!(TWCR & (1<<TWINT)));
    
    // Ladda data i dataregistret
    TWDR = data;
    
    // Starta överföring
    TWCR = (1<<TWINT) | (1<<TWEN);
}