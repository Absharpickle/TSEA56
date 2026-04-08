#include <avr/io.h>
#include <avr/interrupt.h>

#define SLAVE_ADDRESS 0x12
volatile uint8_t rx_buffer[8];
volatile uint8_t rx_index = 0;
volatile uint8_t tx_index = 0; // Index för att skicka tillbaka data

void TWI_init_slave(void) {
    TWAR = (SLAVE_ADDRESS << 1); 
    TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
    sei(); 
}

ISR(TWI_vect) {
    uint8_t status = TWSR & 0xF8;

    // --- SLAVE RECEIVER (Mastern skriver till oss) ---
    if (status == 0x60) { 
        rx_index = 0; // Ny skrivning startar
    } else if (status == 0x80) { 
        if (rx_index < 8) rx_buffer[rx_index++] = TWDR; // Spara data
    } 
    
    // --- SLAVE TRANSMITTER (Mastern läser från oss) ---
    else if (status == 0xA8) { 
        tx_index = 0; // Mastern vill börja läsa
        TWDR = rx_buffer[tx_index++]; // Ladda första byten
    } else if (status == 0xB8) { 
        // Skicka nästa byte om Mastern vill ha mer
        TWDR = (tx_index < 8) ? rx_buffer[tx_index++] : 0x00;
    }

    // Återställ flaggan för att fortsätta kommunicera
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
}

int main(void) {
    TWI_init_slave();
    while (1);
}