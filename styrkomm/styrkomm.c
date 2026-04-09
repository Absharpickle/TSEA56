#define F_CPU 8000000UL // Körs i 8 MHz (ingen CKDIV8)
#include <avr/io.h>
#include <avr/interrupt.h>

#define SLAVE_ADDRESS 0x12

volatile uint8_t rx_buffer[8];
volatile uint8_t rx_index = 0;
volatile uint8_t nytt_paket_mottaget = 0; // Flagga för att ett helt paket kommit

// =================================================================
// MOTOR- OCH STYRFUNKTIONER (Fyll i er PWM-kod här!)
// =================================================================

void kor_framat() {
    // Sätt motorer till att köra framåt
}

void svang_hoger() {
    // Rotera roboten 90 grader åt höger
}

void svang_vanster() {
    // Rotera roboten 90 grader åt vänster
}

void rotera_180_eller_backa() {
    // Rotera 180 grader (eller backa om det är ett hinder)
}

void stanna_motorer() {
    // Stäng av PWM till hjulen (stanna)
}

void plocka_upp_vara() {
    // Styr servon/klo för att plocka upp varan
}

void avlamna_vara() {
    // Styr servon/klo för att släppa varan
}

void pd_reglering_linje(int8_t vinkel_fel) {
    // Använd vinkel_fel för att justera PWM mellan höger och vänster hjul
    // T.ex: 
    // motor_vanster_PWM = BAS_HASTIGHET + (Kp * vinkel_fel);
    // motor_hoger_PWM   = BAS_HASTIGHET - (Kp * vinkel_fel);
}

void reglera_rotationshastighet(int8_t gyro_varde) {
    // Valfritt: Används om ni vill bromsa in svängen när roboten roterar för snabbt
}


// =================================================================
// I2C-KOMMUNIKATION (SLAVE RECEIVER)
// =================================================================

void TWI_init_slave(void) {
    TWAR = (SLAVE_ADDRESS << 1); 
    TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
    sei(); // Aktivera globala interrupts
}

ISR(TWI_vect) {
    uint8_t status = TWSR & 0xF8;

    if (status == 0x60) { 
        // Master (Pi) vill SKRIVA, börja om från början av paketet
        rx_index = 0;
    } else if (status == 0x80) { 
        // Master har skickat en byte
        if (rx_index < 8) {
            rx_buffer[rx_index++] = TWDR;
        }
    } else if (status == 0xA0) { 
        // STOPP-signal från Master. Paketet är färdigt!
        nytt_paket_mottaget = 1; 
    }
    
    // Nollställ I2C-hårdvaran för nästa byte/paket
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
}


// =================================================================
// HUVUDPROGRAM
// =================================================================

int main(void) {
    TWI_init_slave();
    
    uint8_t fall;
    int8_t data_1; // Byte 2
    int8_t data_2; // Byte 3
    int8_t data_3; // Byte 4

    while (1) {
        if (nytt_paket_mottaget) {
            
            // Kolla så det faktiskt är ett giltigt paket från Pi:n
            if (rx_buffer[0] == 0x05 && rx_buffer[7] == 0xFF) {
                
                fall   = rx_buffer[1];
                data_1 = (int8_t)rx_buffer[2]; 
                data_2 = (int8_t)rx_buffer[3]; 
                data_3 = (int8_t)rx_buffer[4]; 

                // --- TOLKA PAKETET ---
                
                if (fall == 1) {
                    // FALL 1: NYTT KOMMANDO
                    char cmd = (char)data_1;
                    
                    switch (cmd) {
                        case 'f': kor_framat(); break;
                        case 'r': svang_hoger(); break;
                        case 'l': svang_vanster(); break;
                        case 'b': rotera_180_eller_backa(); break;
                        case 'v': plocka_upp_vara(); break;
                        case 'a': avlamna_vara(); break;
                        case 's': stanna_motorer(); break;
                        default:  break; // Okänt kommando, gör inget
                    }
                } 
                
                else if (fall == 2) {
                    // FALL 2: LINJEFÖLJNING (Vanlig körning)
                    int8_t vinkel_fel = data_1;
                    int8_t fram_fel   = data_2; // Rådata om ni behöver den
                    int8_t bak_fel    = data_3; // Rådata om ni behöver den
                    
                    // Casta till void för att slippa kompilatorvarningar om ni inte använder dem
                    (void)fram_fel;
                    (void)bak_fel;
                    
                    // Skicka felet till er regulator
                    pd_reglering_linje(vinkel_fel);
                } 
                
                else if (fall == 3) {
                    // FALL 3: ROTATION PÅGÅR
                    int8_t gyro_varde = data_1;
                    
                    // Skicka datan till rotationsregulatorn
                    reglera_rotationshastighet(gyro_varde);
                }
            }
            
            // Markera att vi är klara med detta paket
            nytt_paket_mottaget = 0; 
        }
    }
}