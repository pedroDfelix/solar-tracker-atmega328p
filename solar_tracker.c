// Seguidor solar de 3 eixos (yaw, pitch, roll) - ATmega328P, 16 MHz
// LABSI, ISEP 2025/2026 - Pedro Deveza Félix e Pedro Miguel Almeida

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>

// pitch (PD3)
#define PITCH_MIN_ANG        0
#define PITCH_MAX_ANG        180
#define PITCH_CENTER         165

// largura do impulso em us
#define PITCH_MIN_US         1200u
#define PITCH_MAX_US         1800u

// passo do pitch em graus
#define PITCH_STEP           3u

// histerese para sair do limite
#define PITCH_RELEASE_BAND   15

// roll (PB2)
#define ROLL_MIN_ANG         0
#define ROLL_MAX_ANG         90
#define ROLL_CENTER          85

// yaw (PB1), servo de rotação contínua
#define YAW_STOP_US          1450u
#define SPEED_US_DELTA       60u
#define MOVE_TIME_MS         580u
#define DIR  (+1)

// tempos (ms)
#define DURATION_POINT_X     2000
#define DURATION_RECOVER     1000
#define DURATION_NIGHT_ROT   4000

// LDR
#define NIGHT_THRESHOLD      250
#define DEAD_BAND            40
#define LOOP_INTERVAL_MS     50

typedef enum {
    STATE_TRACKING,
    STATE_NIGHT_PREPARE
} SystemState;

volatile SystemState current_state = STATE_TRACKING;
volatile uint32_t system_millis = 0;
uint8_t sistema_invertido = 0;

volatile uint8_t curr_pitch = PITCH_CENTER;
volatile uint8_t curr_roll  = ROLL_CENTER;
uint32_t state_start_time = 0;
uint32_t last_loop_time = 0;

// pitch: impulso em ticks de 20us
volatile uint16_t pitch_pulse_20us = 75; // 1500us

// Timer0 - millis

void timer0_init(void) {
    TCCR0A = (1 << WGM01); // CTC
    OCR0A  = 249;          // 1ms a 16MHz/64
    TCCR0B = (1 << CS01) | (1 << CS00);
    TIMSK0 |= (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect) { system_millis++; }

uint32_t millis(void) {
    uint32_t m;
    uint8_t s = SREG; cli(); m = system_millis; SREG = s;
    return m;
}

static void wait_ms(uint16_t ms) {
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < ms) {}
}

// servos

// Timer1: PWM de 50Hz no yaw (PB1) e no roll (PB2)
void timer1_init(void) {
    DDRB |= (1 << PB1) | (1 << PB2);
    TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 39999; // 20ms
}

uint16_t angle_to_t1(uint8_t ang) {
    return (uint16_t)(2000u + ((uint32_t)2000u * ang) / 180u);
}

static inline void yaw_set_pulse(uint16_t us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    OCR1A = (uint16_t)(2u * us);
}
void set_roll(uint8_t ang) { OCR1B = angle_to_t1(ang); }

// Timer2: PWM por software no pitch (PD3)
void timer2_init_pitch(void) {
    DDRD |= (1 << PD3);

    TCCR2A = (1 << WGM21);          // CTC
    TCCR2B = (1 << CS21);           // prescaler 8 -> tick=0.5us
    OCR2A  = 39;                    // (39+1)*0.5us = 20us
    TIMSK2 |= (1 << OCIE2A);
}

ISR(TIMER2_COMPA_vect) {
    static uint16_t tick20 = 0; // 0..999 (20ms)

    if (tick20 == 0) PORTD |= (1 << PD3);
    if (tick20 == pitch_pulse_20us) PORTD &= ~(1 << PD3);

    tick20++;
    if (tick20 >= 1000) tick20 = 0; // 20ms
}

static inline uint16_t us_to_20us_ticks(uint16_t us) {
    uint16_t t = (us + 10u) / 20u;   // arredonda
    if (t < 25)  t = 25;             // 500us
    if (t > 125) t = 125;            // 2500us
    return t;
}

void set_pitch(uint8_t ang) {
    if (ang > 180) ang = 180;

    uint16_t us = (uint16_t)(PITCH_MIN_US
                + (uint32_t)(PITCH_MAX_US - PITCH_MIN_US) * ang / 180u);

    uint16_t t20 = us_to_20us_ticks(us);

    uint8_t s = SREG; cli();
    pitch_pulse_20us = t20;
    SREG = s;
}

// yaw
void yaw_stop(void) { yaw_set_pulse(YAW_STOP_US); }
void yaw_180(void) {
    yaw_set_pulse((uint16_t)(YAW_STOP_US + (DIR * (int16_t)SPEED_US_DELTA)));
    wait_ms(MOVE_TIME_MS);
    yaw_set_pulse(YAW_STOP_US);
}

// LCD 20x4 por I2C

#define I2C_FREQ_HZ             100000UL
#define LCD_I2C_ADDR_DEFAULT    0x27u

#define LCD_BIT_RS              (1u<<0)
#define LCD_BIT_E               (1u<<2)
#define LCD_BIT_BL              (1u<<3)

#define LCD_CMD_CLEAR           0x01u
#define LCD_CMD_HOME            0x02u
#define LCD_CMD_ENTRYMODE       0x06u
#define LCD_CMD_DISP_ON         0x0Cu
#define LCD_CMD_FUNC_4BIT       0x28u

#define LCD_ROW0_ADDR           0x00u
#define LCD_ROW1_ADDR           0x40u
#define LCD_ROW2_ADDR           0x14u
#define LCD_ROW3_ADDR           0x54u

#define LCD_T_INIT_MS           50u
#define LCD_T_CMD_MS            2u
#define LCD_T_CLEAR_MS          2u

static uint8_t lcd_addr = LCD_I2C_ADDR_DEFAULT;
static uint8_t lcd_bl_state = LCD_BIT_BL;

// I2C (TWI)
static void init_i2c(void) {
    TWSR = 0x00;
    TWBR = (uint8_t)((F_CPU / I2C_FREQ_HZ - 16u) / 2u);
    TWCR = (1u << TWEN);
}

static uint8_t i2c_start_write(uint8_t addr7) {
    TWCR = (1u << TWINT) | (1u << TWSTA) | (1u << TWEN);
    while (!(TWCR & (1u << TWINT))) {}

    TWDR = (uint8_t)((addr7 << 1) | 0u);
    TWCR = (1u << TWINT) | (1u << TWEN);
    while (!(TWCR & (1u << TWINT))) {}

    return ((TWSR & 0xF8u) == 0x18u);
}

static void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1u << TWINT) | (1u << TWEN);
    while (!(TWCR & (1u << TWINT))) {}
}

static void i2c_stop(void) {
    TWCR = (1u << TWINT) | (1u << TWEN) | (1u << TWSTO);
}

// LCD (PCF8574)
static void lcd_pcf_write(uint8_t v) {
    if (i2c_start_write(lcd_addr)) {
        i2c_write((uint8_t)(v | lcd_bl_state));
    }
    i2c_stop();
}

static void lcd_pulse(uint8_t v) {
    lcd_pcf_write((uint8_t)(v | LCD_BIT_E));
    lcd_pcf_write((uint8_t)(v & (uint8_t)~LCD_BIT_E));
}

static void lcd_write4(uint8_t nibble, uint8_t rs) {
    uint8_t out = (uint8_t)((nibble & 0x0Fu) << 4);
    if (rs) out |= LCD_BIT_RS;
    lcd_pulse(out);
}

static void lcd_cmd(uint8_t cmd) {
    lcd_write4((uint8_t)(cmd >> 4), 0);
    lcd_write4((uint8_t)(cmd & 0x0F), 0);
    if (cmd == LCD_CMD_CLEAR || cmd == LCD_CMD_HOME) wait_ms(LCD_T_CLEAR_MS);
    else wait_ms(LCD_T_CMD_MS);
}

static void lcd_data(uint8_t data) {
    lcd_write4((uint8_t)(data >> 4), 1);
    lcd_write4((uint8_t)(data & 0x0F), 1);
}

static void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t base;
    switch (row) {
        default:
        case 0: base = LCD_ROW0_ADDR; break;
        case 1: base = LCD_ROW1_ADDR; break;
        case 2: base = LCD_ROW2_ADDR; break;
        case 3: base = LCD_ROW3_ADDR; break;
    }
    lcd_cmd((uint8_t)(0x80u | (base + col)));
}

static void lcd_print_str(const char *s) {
    while (*s) lcd_data((uint8_t)*s++);
}

static void lcd_print_u16(uint16_t v) {
    char buf[6];
    uint8_t i = 0;
    if (v == 0) { lcd_data('0'); return; }
    while (v && i < 5) { buf[i++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (i) lcd_data((uint8_t)buf[--i]);
}

static uint8_t i2c_scan_first_addr(void) {
    for (uint8_t a = 0x03u; a <= 0x77u; a++) {
        if (i2c_start_write(a)) { i2c_stop(); return a; }
        i2c_stop();
    }
    return 0u;
}

static void lcd_init_driver(void) {
    wait_ms(LCD_T_INIT_MS);

    lcd_write4(0x03, 0); wait_ms(5);
    lcd_write4(0x03, 0); wait_ms(2);
    lcd_write4(0x03, 0); wait_ms(2);
    lcd_write4(0x02, 0); wait_ms(2);

    lcd_cmd(LCD_CMD_FUNC_4BIT);
    lcd_cmd(LCD_CMD_DISP_ON);
    lcd_cmd(LCD_CMD_ENTRYMODE);
    lcd_cmd(LCD_CMD_CLEAR);
}

static void lcd_init(void) { lcd_init_driver(); }
static void lcd_command(uint8_t c) { lcd_cmd(c); }
static void lcd_puts(const char *s) { lcd_print_str(s); }
static void lcd_print_num(uint16_t v) { lcd_print_u16(v); }
static void lcd_gotoxy(uint8_t col, uint8_t row) { lcd_set_cursor(row, col); }

// ADC
uint16_t read_adc(uint8_t ch) {
    ADMUX = (1<<REFS0) | (ch & 0x07);
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    return ADC;
}

// seguimento
void process_tracking(uint16_t t, uint16_t b, uint16_t l, uint16_t r) {

    // pitch: cima/baixo
    static uint8_t pitch_sat = 0;

    int16_t e_tb = (int16_t)t - (int16_t)b; // >0: mais luz em cima

    if (pitch_sat) {
        if (abs(e_tb) < PITCH_RELEASE_BAND) {
            pitch_sat = 0;
        } else if ((curr_pitch >= PITCH_MAX_ANG && e_tb < 0) ||
                   (curr_pitch <= PITCH_MIN_ANG && e_tb > 0)) {
            pitch_sat = 0;
        } else {
            // ainda no limite
            goto ROLL_PART;
        }
    }

    if (abs(e_tb) > DEAD_BAND) {
        if (sistema_invertido == 0) {
            // modo normal
            if (e_tb > 0) { // t > b
                if (curr_pitch < PITCH_MAX_ANG) {
                    uint8_t next = curr_pitch + PITCH_STEP;
                    curr_pitch = (next <= PITCH_MAX_ANG) ? next : PITCH_MAX_ANG;
                    set_pitch(curr_pitch);
                } else {
                    yaw_180();
                    curr_pitch = PITCH_CENTER - 5;
                    set_pitch(curr_pitch);
                    pitch_sat = 1;
                }
            } else { // t < b
                if (curr_pitch > PITCH_MIN_ANG) {
                    uint8_t next = (curr_pitch >= (PITCH_MIN_ANG + PITCH_STEP)) ?
                                   (uint8_t)(curr_pitch - PITCH_STEP) : PITCH_MIN_ANG;
                    curr_pitch = next;
                    set_pitch(curr_pitch);
                } else {
                    pitch_sat = 1;
                }
            }
        } else {
            // modo invertido
            if (e_tb > 0) { // luz em cima -> desce
                if (curr_pitch > PITCH_MIN_ANG) {
                    uint8_t next = (curr_pitch >= (PITCH_MIN_ANG + PITCH_STEP)) ?
                                   (uint8_t)(curr_pitch - PITCH_STEP) : PITCH_MIN_ANG;
                    curr_pitch = next;
                    set_pitch(curr_pitch);
                } else {
                    pitch_sat = 1;
                }
            } else { // luz em baixo -> sobe
                if (curr_pitch < PITCH_MAX_ANG) {
                    uint8_t next = curr_pitch + PITCH_STEP;
                    curr_pitch = (next <= PITCH_MAX_ANG) ? next : PITCH_MAX_ANG;
                    set_pitch(curr_pitch);
                } else {
                    yaw_180();
                    curr_pitch = PITCH_CENTER - 5;
                    set_pitch(curr_pitch);
                    pitch_sat = 1;
                }
            }
        }
    }

ROLL_PART:
    // roll: esquerda/direita
    if (sistema_invertido == 0) {
        if (abs((int)l - (int)r) > DEAD_BAND) {
            if (l > r) {
                if (curr_roll < ROLL_MAX_ANG) {
                    curr_roll++;
                    set_roll(curr_roll);
                } else {
                    yaw_180();
                    curr_roll = ROLL_CENTER - 5;
                    set_roll(curr_roll);
                }
            } else {
                if (curr_roll > ROLL_MIN_ANG) {
                    curr_roll--;
                    set_roll(curr_roll);
                }
            }
        }
    } else {
        if (abs((int)l - (int)r) > DEAD_BAND) {
            if (l > r) {
                if (curr_roll > ROLL_MIN_ANG) {
                    curr_roll--;
                    set_roll(curr_roll);
                }
            } else {
                if (curr_roll < ROLL_MAX_ANG) {
                    curr_roll++;
                    set_roll(curr_roll);
                } else {
                    yaw_180();
                    curr_roll = ROLL_CENTER - 5;
                    set_roll(curr_roll);
                }
            }
        }
    }
}

int main (void) {
    // ADC init
    ADMUX = (1<<REFS0);
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

    timer0_init();
    timer1_init();
    timer2_init_pitch();

    init_i2c();
    sei();

    uint8_t found = i2c_scan_first_addr();
    if (found) lcd_addr = found;

    lcd_init();

    set_pitch(curr_pitch);
    set_roll(curr_roll);
    yaw_stop();

    lcd_command(0x01);
    lcd_gotoxy(0,0);
    lcd_puts("Solar Panel Tracker");

    while (1) {
        uint32_t now = millis();

        if (now - last_loop_time >= LOOP_INTERVAL_MS) {
            last_loop_time = now;

            uint16_t tl = 1023 - read_adc(0);
            uint16_t tr = 1023 - read_adc(1);
            uint16_t bl = 1023 - read_adc(2);
            uint16_t br = 1023 - read_adc(3);

            uint16_t avg_tot = (tl+tr+bl+br)/4;
            uint16_t avg_top = (tl+tr)/2;
            uint16_t avg_bot = (bl+br)/2;
            uint16_t avg_l   = (tl+bl)/2;
            uint16_t avg_r   = (tr+br)/2;

            switch (current_state) {
                case STATE_TRACKING:
                    if (avg_tot < NIGHT_THRESHOLD) {
                        current_state = STATE_NIGHT_PREPARE;
                        state_start_time = now;
                    } else {
                        process_tracking(avg_top, avg_bot, avg_l, avg_r);

                        lcd_gotoxy(0,0);
                        lcd_puts("Solar Panel Tracker");

                        lcd_gotoxy(0,1);
                        lcd_puts("L1:"); lcd_print_num(tl);
                        lcd_puts(" L2:"); lcd_print_num(tr);
                        lcd_puts("   ");

                        lcd_gotoxy(0,2);
                        lcd_puts("L3:"); lcd_print_num(bl);
                        lcd_puts(" L4:"); lcd_print_num(br);
                        lcd_puts("   ");

                        lcd_gotoxy(0,3);
                        lcd_puts("AVG:"); lcd_print_num(avg_tot);
                        lcd_puts(" P:"); lcd_print_num(curr_pitch);
                        lcd_puts(" R:"); lcd_print_num(curr_roll);
                        lcd_puts("   ");
                    }
                    break;

                case STATE_NIGHT_PREPARE:
                    lcd_gotoxy(0,0); lcd_puts("MODO NOITE      ");

                    if (avg_tot < NIGHT_THRESHOLD) {
                        yaw_180();
                        yaw_stop();
                        curr_pitch = PITCH_CENTER - 5;
                        curr_roll  = ROLL_CENTER - 5;
                        set_pitch(curr_pitch);
                        set_roll(curr_roll);
                    }

                    if (avg_tot >= (NIGHT_THRESHOLD + 50)) {
                        current_state = STATE_TRACKING;
                        lcd_command(0x01);
                        lcd_gotoxy(0,0);
                        lcd_puts("Solar Panel Tracker");
                    }
                    break;
            }
        }
    }
}
