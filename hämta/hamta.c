#include <avr/io.h>

#define SLAVE_ADDRESS 0x12

// Globala variabler för att spara inkommande data
volatile uint8_t rx_buffer[8];
volatile uint8_t rx_index = 0;

void TWI_init_slave(void) {
    TWAR = (SLAVE_ADDRESS << 1); 
    // Aktivera TWI och svara med ACK
    TWCR = (1<<TWEN) | (1<<TWEA);
}

void TWI_handle_communication(void) {
    // 1. Vänta på att något händer på bussen
    while (!(TWCR & (1<<TWINT)));

    // 2. Kolla statusregistret
    uint8_t status = TWSR & 0xF8;

    if (status == 0x60) {
        // 0x60: Master har ropat på vår adress och vill SKRIVA till oss
        rx_index = 0; // Nollställ pekaren för en ny överföring
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    } 
    else if (status == 0x80) {
        // 0x80: Vi har tagit emot en byte med data
        if (rx_index < 8) {
            rx_buffer[rx_index] = TWDR; // Spara datan från dataregistret
            rx_index++;
        }
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA); // Svara ACK på nästa byte
    } 
    else if (status == 0xA0) {
        // 0xA0: Master har skickat STOP-signal (överföring klar)
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    } 
    else {
        // Om något annat händer (t.ex. fel), återställ bussen
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    }
}

int main(void) {
    TWI_init_slave();

    while (1) {
        // Loopen väntar ständigt på inkommande data
        TWI_handle_communication();
        
        // Här kan du läsa från rx_buffer[0] till rx_buffer[7]
        // och göra vad du vill med datan när en överföring är klar.
    }
    
    return 0;
}