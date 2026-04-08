#include <avr/io.h>
#include <avr/interrupt.h> // Viktigt: Biblioteket för avbrott (Interrupts)

#define SLAVE_ADDRESS 0x11

// Datan vi simulerar från tabellen (8 bytes)
volatile uint8_t tx_buffer[8] = {0x05, 0xF1, 0xFF, 0x14, 0x00, 35, 0x01, 0xFF};
volatile uint8_t tx_index = 0;

void TWI_init_slave(void) {
    TWAR = (SLAVE_ADDRESS << 1); 
    // Aktivera TWI, svara med ACK, och aktivera INTERRUPTS (TWIE)
    TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE); 
    sei(); // Aktivera globala avbrott
}

// Denna funktion triggas automatiskt av hårdvaran när Raspberry Pi vill prata!
ISR(TWI_vect) {
    uint8_t status = TWSR & 0xF8;

    if (status == 0xA8) {
        // 0xA8: Master vill LÄSA från oss (SLA+R mottagen)
        tx_index = 0; 
        TWDR = tx_buffer[tx_index++]; // Ladda första byten i dataregistret
        
        // Rensa flaggan och behåll TWIE (Interrupt Enable) aktivt
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
    } 
    else if (status == 0xB8) {
        // 0xB8: Data har skickats, Master svarade ACK och vill ha NÄSTA byte
        if (tx_index < 8) {
            TWDR = tx_buffer[tx_index++];
        } else {
            TWDR = 0x00; // Skicka nollor om Master begär fler än 8 bytes
        }
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
    } 
    else if (status == 0xC0 || status == 0xC8) {
        // 0xC0: Data skickad, NACK mottaget (Raspberry Pi har fått det den vill ha och stänger)
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
    }
    else {
        // Återställ bussen vid andra oförutsedda tillstånd
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
    }
}

int main(void) {
    TWI_init_slave();
    
    while (1) {
        // Huvudloopen är nu helt tom! 
        // Processorn kan sova eller göra andra saker, I2C sköts helt i bakgrunden.
    }
    return 0;
}