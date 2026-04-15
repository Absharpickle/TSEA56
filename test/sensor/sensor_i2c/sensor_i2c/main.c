/* ==========================================================================
 * Sensormodul - ATmega1284P @ 16 MHz extern kristall
 * TSEA56 Kandidatprojekt, Projektgrupp 3
 *
 * Denna modul läser av:
 * - 2x12 linjesensorer via multiplexrar
 * - 1x IR-avståndsmätare (analog)
 * - 1x Gyro (SPI)
 *
 * Data skickas som slav via I2C (TWI) till en master (t.ex. Raspberry Pi).
 * ========================================================================== */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>

/* --------------------------------------------------------------------------
 * Konfiguration & Definitioner
 * -------------------------------------------------------------------------- */
#define NUM_LINE_SENSORS    12
#define SENSOR_DISTANCE_MM  70
#define LINE0_ADC_CH        0
#define LINE1_ADC_CH        1
#define IR_ADC_CH           2

/* Korsning eller upphämtning */
#define TIME_WINDOW_TICKS   30
#define PICKUP_MIN          5
#define PICKUP_MAX          8
#define INTERSECT_MIN       9

/* IIR-filter parametrar */
#define IIR_ALPHA_NUM       26u   // 26/256 = ~0.10
#define IIR_SCALE           256u
#define GYRO_IIR_ALPHA_NUM  96u   // ~0.38

/* Pin-konfiguration */
#define MUX_PORT            PORTD
#define MUX_DDR             DDRD
#define MUX_A0              PD0
#define MUX_A1              PD1
#define MUX_A2              PD2
#define MUX_A3              PD3
#define LINE0_EN            PD4
#define LINE1_EN            PD5

#define GYRO_PORT           PORTB
#define GYRO_DDR            DDRB
#define GYRO_CS             PB4
#define GYRO_MOSI           PB5
#define GYRO_MISO           PB6
#define GYRO_SCK            PB7

#define I2C_SLAVE_ADDR      0x10

/* Timer0 CTC, prescaler 1024 -> tick = 64 us @ 16 MHz
 * OCR0A = 155 -> 156 x 64 us = 9984 us ~ 10 ms
 */
#define TIMER0_OCR          155

#define LINE_SPACING_MM     10
#define LINE0_THRESHOLD     70    // Förmodligen höj till arena
#define LINE1_THRESHOLD     70
#define INVALID_MM          0x7FFF

#define OBSTACLE_CM         90
#define PKT_LEN             8

/* Stadier vid linjedetektion */
typedef enum {
    FEATURE_NONE         = 0,
    FEATURE_PICKUP       = 1,
    FEATURE_INTERSECTION = 2
} line_feature_t;

/* --------------------------------------------------------------------------
 * Globala variabler
 * -------------------------------------------------------------------------- */
static volatile uint16_t line0_raw[NUM_LINE_SENSORS];
static volatile uint16_t line1_raw[NUM_LINE_SENSORS];
static volatile uint16_t ir_raw;

/* Filtrerade värden */
static uint16_t line0_LPF[NUM_LINE_SENSORS];
static uint16_t line1_LPF[NUM_LINE_SENSORS];
static volatile int16_t gyro_omega_lpf = 0;

/* Filterminne */
static uint32_t line0_acc[NUM_LINE_SENSORS];
static uint32_t line1_acc[NUM_LINE_SENSORS];
static int32_t  gyro_acc = 0;
static uint8_t  gyro_lpf_init = 0;

/* ADC-tillstånd (används i ISR) */
static volatile uint8_t mux_ch      = 0;
static volatile uint8_t adc_phase   = 0;   /* 0=dummy, 1=riktig */
static volatile uint8_t sweep_stage = 0;   /* 0=linje0, 1=linje1, 2=IR */
static volatile uint8_t sweep_done  = 0;

/* I2C-sändbuffert */
static volatile uint8_t tx_buf[PKT_LEN];
static volatile uint8_t tx_idx = 0;

/* Debug/Watch-variabler */
static volatile int16_t line0_dev_dbg = 0;
static volatile int16_t line1_dev_dbg = 0;
static volatile int16_t gyro_omega = 0;
static volatile int16_t line_angle_dbg = 0;
static volatile int16_t dist_dbg = 0;
static volatile uint8_t feature_dbg = 0;
static volatile uint8_t black_count_dbg = 0;
static volatile uint8_t stat_dbg = 0;

/* --------------------------------------------------------------------------
 * Hjälpfunktioner: MUX & ADC
 * -------------------------------------------------------------------------- */
static void mux_select(uint8_t ch) {
    MUX_PORT = (MUX_PORT & 0xF0) | (ch & 0x0F);
}

static void adc_init(void) {
    ADMUX  = (1 << REFS0); // AVcc som referens
    DIDR0  = (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D);
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Div 128
}

static void adc_start(uint8_t ch) {
    ADMUX  = (ADMUX & 0xF0) | (ch & 0x07);
    ADCSRA |= (1 << ADSC);
}

/* --------------------------------------------------------------------------
 * Beräkningar: Linjesensor & IR
 * -------------------------------------------------------------------------- */
static int16_t line_centroid(const uint16_t *raw, uint16_t threshold) {
    int16_t sum_w   = 0;
    int32_t sum_pos = 0;

    for (uint8_t i = 1; i < NUM_LINE_SENSORS; i++) {
        if (raw[i] > threshold) {
            int16_t pos_mm = (int16_t)(i - (NUM_LINE_SENSORS / 2)) * LINE_SPACING_MM;
            sum_w   += 1;
            sum_pos += pos_mm;
        }
    }

    if (sum_w == 0) return (int16_t)INVALID_MM;
    return (int16_t)(sum_pos / sum_w);
}

/* Orienteringsfelet */
static int16_t line_angle_tenths_deg(int16_t c0, int16_t c1, float d_mm) {
    if (c0 == (int16_t)INVALID_MM || c1 == (int16_t)INVALID_MM || d_mm <= 0.0f) {
        return 0;
    }

    float angle_rad = atanf(((float)c0 - (float)c1) / d_mm);
    float angle_deg = angle_rad * 57.29578f;

    return (int16_t)(angle_deg * 10.0f);
}

static uint8_t ir_to_cm(uint16_t adc) {
    static const uint16_t table[][2] = {
        {481, 20}, {394, 30}, {311, 40}, {257, 50},
        {225, 60}, {199, 70}, {178, 80}, {170, 90}, {160, 100},
        {153, 110}
    };
    const uint8_t len = 10;

    if (adc >= table[0][0]) return (uint8_t)table[0][1];

    for (uint8_t i = 1; i < len; i++) {
        if (adc >= table[i][0]) {
            uint16_t a0 = table[i - 1][0];
            uint16_t a1 = table[i][0];
            uint8_t  d0 = (uint8_t)table[i - 1][1];
            uint8_t  d1 = (uint8_t)table[i][1];
            return d0 + (uint8_t)((uint32_t)(d1 - d0) * (a0 - adc) / (a0 - a1));
        }
    }
    return 150;
}

static line_feature_t detect_line_feature(const uint16_t *lpf, uint16_t threshold) {
    static line_feature_t held_feature = FEATURE_NONE;
    static uint8_t ticks_remaining = 0;

    uint8_t black_count = 0;
    for (uint8_t i = 1; i < NUM_LINE_SENSORS; i++) {
        if (lpf[i] > threshold) {
            black_count++;
        }
    }
    black_count_dbg = black_count;

    line_feature_t current = FEATURE_NONE;
    if (black_count >= INTERSECT_MIN) {
        current = FEATURE_INTERSECTION;
    } else if (black_count >= PICKUP_MIN && black_count <= PICKUP_MAX) {
        current = FEATURE_PICKUP;
    }

    if (current != FEATURE_NONE) {
        if (current == FEATURE_INTERSECTION || held_feature != FEATURE_INTERSECTION) {
            held_feature = current;
        }
        ticks_remaining = TIME_WINDOW_TICKS;
    } else if (ticks_remaining > 0) {
        ticks_remaining--;
    } else {
        held_feature = FEATURE_NONE;
    }
    
    feature_dbg = (uint8_t)held_feature;
    return held_feature;
}

/* --------------------------------------------------------------------------
 * SPI & Gyro
 * -------------------------------------------------------------------------- */
static void spi_init(void) {
    GYRO_DDR |= (1 << GYRO_MOSI) | (1 << GYRO_SCK) | (1 << GYRO_CS);
    GYRO_PORT |= (1 << GYRO_CS);
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0); // Prescaler /16
}

static uint16_t spi_transfer_16(uint8_t instruction) {
    uint16_t result = 0;
    GYRO_PORT &= ~(1 << GYRO_CS);
    
    SPDR = instruction;
    while (!(SPSR & (1 << SPIF)));
    
    SPDR = 0x00;
    while (!(SPSR & (1 << SPIF)));
    result = (uint16_t)SPDR << 8;

    SPDR = 0x00;
    while (!(SPSR & (1 << SPIF)));
    result |= SPDR;

    GYRO_PORT |= (1 << GYRO_CS);
    return result;
}

static void gyro_init(void) {
    uint16_t resp;
    uint8_t attempts = 0;
    do {
        resp = spi_transfer_16(0x94);
        attempts++;
        _delay_us(200);
    } while ((resp >> 15) && (attempts < 50));
    _delay_us(120);
}

static int16_t gyro_read_omega(void) {
    uint16_t raw;
    spi_transfer_16(0x94);
    _delay_us(120);
    raw = spi_transfer_16(0x80);

    if (raw >> 15) return 0; // Fel/Busy

    int16_t adc = (int16_t)((raw >> 1) & 0x07FF);
    return (int16_t)((adc - 1016) * 10 / 64);
}

/* --------------------------------------------------------------------------
 * Filteruppdateringar
 * -------------------------------------------------------------------------- */
static void iir_lpf_update_all(void) {
    for (uint8_t i = 0; i < NUM_LINE_SENSORS; i++) {
        line0_acc[i] = (uint32_t)IIR_ALPHA_NUM * line0_raw[i] + (IIR_SCALE - IIR_ALPHA_NUM) * line0_acc[i] / IIR_SCALE;
        line0_LPF[i] = (uint16_t)(line0_acc[i] / IIR_SCALE);

        line1_acc[i] = (uint32_t)IIR_ALPHA_NUM * line1_raw[i] + (IIR_SCALE - IIR_ALPHA_NUM) * line1_acc[i] / IIR_SCALE;
        line1_LPF[i] = (uint16_t)(line1_acc[i] / IIR_SCALE);
    }
}

static int16_t gyro_lpf_update(int16_t x) {
    if (!gyro_lpf_init) {
        gyro_acc = (int32_t)x * IIR_SCALE;
        gyro_lpf_init = 1;
    } else {
        gyro_acc = ((int32_t)(IIR_SCALE - GYRO_IIR_ALPHA_NUM) * gyro_acc + (int32_t)GYRO_IIR_ALPHA_NUM * ((int32_t)x * IIR_SCALE)) / IIR_SCALE;
    }
    return (int16_t)(gyro_acc / IIR_SCALE);
}

/* --------------------------------------------------------------------------
 * Interrupt Service Routines (ISR)
 * -------------------------------------------------------------------------- */
ISR(TIMER0_COMPA_vect) {
    mux_ch = 0; 
    adc_phase = 0; 
    sweep_stage = 0;
    mux_select(0);
    adc_start(LINE0_ADC_CH);
}

ISR(ADC_vect) {
    uint16_t val = ADCW;
    if (sweep_stage == 0) { // Linje 0
        if (adc_phase == 0) { 
            adc_phase = 1; 
            adc_start(LINE0_ADC_CH); 
        } else {
            line0_raw[mux_ch] = val;
            if (++mux_ch < NUM_LINE_SENSORS) { 
                mux_select(mux_ch); 
                adc_phase = 0; 
                adc_start(LINE0_ADC_CH); 
            } else { 
                mux_ch = 0; 
                adc_phase = 0; 
                sweep_stage = 1; 
                mux_select(0); 
                adc_start(LINE1_ADC_CH); 
            }
        }
    } else if (sweep_stage == 1) { // Linje 1
        if (adc_phase == 0) { 
            adc_phase = 1; 
            adc_start(LINE1_ADC_CH); 
        } else {
            line1_raw[mux_ch] = val;
            if (++mux_ch < NUM_LINE_SENSORS) { 
                mux_select(mux_ch); 
                adc_phase = 0; 
                adc_start(LINE1_ADC_CH); 
            } else { 
                sweep_stage = 2; 
                adc_start(IR_ADC_CH); 
            }
        }
    } else { // IR
        ir_raw = val;
        sweep_done = 1;
    }
}

ISR(TWI_vect) {
    switch (TWSR & 0xF8) {
        case 0xA8: // Address matched
            tx_idx = 0;
            TWDR = tx_buf[tx_idx++];
            break;
        case 0xB8: // Data transmitted, ACK received
            TWDR = (tx_idx < PKT_LEN) ? tx_buf[tx_idx++] : 0xFF;
            break;
    }
    TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
}

/* --------------------------------------------------------------------------
 * Main init & loop
 * -------------------------------------------------------------------------- */
static void pkt_write_i16(uint8_t idx, int16_t v) {
    tx_buf[idx] = (uint8_t)(v & 0xFF);
    tx_buf[idx + 1] = (uint8_t)((v >> 8) & 0xFF);
}

int main(void) {
    // I/O Init
    MUX_DDR |= (1 << MUX_A0) | (1 << MUX_A1) | (1 << MUX_A2) | (1 << MUX_A3) | (1 << LINE0_EN) | (1 << LINE1_EN);
    MUX_PORT |= (1 << LINE0_EN) | (1 << LINE1_EN);

    // Modul Init
    adc_init();
    spi_init();
    gyro_init();
    
    // TWI Init
    TWAR = (I2C_SLAVE_ADDR << 1);
    TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWIE);

    // Timer0 Init
    TCCR0A = (1 << WGM01); // CTC
    TCCR0B = (1 << CS02) | (1 << CS00); // 1024 prescaler
    OCR0A = TIMER0_OCR;
    TIMSK0 |= (1 << OCIE0A);

    sei();

    while (1) {
        if (sweep_done) {
            sweep_done = 0;
            iir_lpf_update_all();
            
            line_feature_t feature = detect_line_feature(line1_LPF, LINE1_THRESHOLD);

            int16_t dev0 = line_centroid(line0_LPF, LINE0_THRESHOLD);
            int16_t dev1 = line_centroid(line1_LPF, LINE1_THRESHOLD);
            uint8_t dist = ir_to_cm(ir_raw);
            dist_dbg = dist;
            
            int16_t omega = gyro_read_omega();
            int16_t angle = line_angle_tenths_deg(dev0, dev1, SENSOR_DISTANCE_MM);
            line_angle_dbg = angle;

            // Uppdatera sändbuffert säkert
            uint8_t stat = 0;
            if (dev0 != (int16_t)INVALID_MM) stat |= (1 << 0);
            if (dev1 != (int16_t)INVALID_MM) stat |= (1 << 1);
            
            stat |= ((uint8_t)feature & 0x03) << 2;
            if (dist <= OBSTACLE_CM) stat |= (1 << 4);
            stat_dbg = stat;

            cli(); // Avaktivera interrupts under skrivning till buffert
            tx_buf[0] = stat;
            pkt_write_i16(1, dev0);
            pkt_write_i16(3, dev1);
            tx_buf[5] = dist;
            pkt_write_i16(6, omega);
            sei();

            // Debug-uppdatering
            line0_dev_dbg = dev0;
            line1_dev_dbg = dev1;
            gyro_omega = omega;
        }
    }
}