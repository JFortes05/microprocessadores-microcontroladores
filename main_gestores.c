/*
 * Trabalho 6 - MCU A / Gestor de Sinalizacao
 * ATmega328P - Microchip Studio / AVR-GCC
 * F_CPU = 16 MHz
 *
 * Cumpre:
 * - Timer1 PWM em PB1 / OC1A
 * - Timer2 CTC para temporizacao do semaforo
 * - INT1 em PD3 para pedido de peao
 * - USART RX em PD0 para receber dados do MCU B
 * - TWI/I2C mestre em PC4/PC5
 *
 * Pinagem adaptada a tua montagem:
 * D4 = verde
 * D6 = amarelo
 * D5 = vermelho
 * B1 = PWM/brilho
 * D7 = baixa visibilidade
 * B0 = falha/emergencia
 * C3 = pedido de peao
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdint.h>

/* =========================
   PROTOCOLO ENTRE MCUs
   ========================= */

#define MCU_B_ADDR              0x12

#define I2C_CMD_POLL            0x20
#define I2C_CMD_PED_REQUEST     0x21

#define FRAME_SOF               0xA5

#define MSG_VISIBILITY          0x10
#define MSG_TRAFFIC             0x11
#define MSG_PED_RESULT          0x12
#define MSG_EMERGENCY           0x13

#define VIS_DARK                0u
#define VIS_NORMAL              1u
#define VIS_BRIGHT              2u

#define TRAFFIC_LOW             0u
#define TRAFFIC_MEDIUM          1u
#define TRAFFIC_HIGH            2u

#define PED_DENIED              0u
#define PED_ACCEPTED            1u

#define EMERGENCY_OFF           0u
#define EMERGENCY_ON            1u

/* =========================
   PINOS MCU A
   ========================= */

#define LED_GREEN               PD4
#define LED_YELLOW              PD6
#define LED_RED                 PD5
#define LED_LOW_VIS             PD7

#define LED_FAULT               PB0
#define LED_PWM                 PB1

#define LED_PED                 PC3

#define BTN_PED                 PD3
#define BTN_RESET_PED           PC0
#define BTN_LOW_VIS             PC1
#define BTN_FAULT               PC2

/* =========================
   ESTADOS
   ========================= */

typedef enum
{
    STATE_GREEN = 0,
    STATE_YELLOW,
    STATE_RED
} traffic_state_t;

volatile traffic_state_t state = STATE_GREEN;

volatile uint16_t tick_state = 0;
volatile uint16_t tick_poll = 0;

volatile uint16_t time_green = 300;     /* 3 s */
volatile uint16_t time_yellow = 120;    /* 1.2 s */
volatile uint16_t time_red = 300;       /* 3 s */

volatile uint8_t ped_request = 0;
volatile uint8_t emergency = 0;
volatile uint8_t low_visibility = 0;
volatile uint8_t need_i2c_ped = 0;

/* =========================
   BUFFER USART
   ========================= */

#define RX_BUF_SIZE 32

volatile uint8_t rx_buf[RX_BUF_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;

static uint8_t rx_available(void)
{
    return rx_head != rx_tail;
}

static uint8_t rx_get(void)
{
    uint8_t data = 0;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (rx_head != rx_tail)
        {
            data = rx_buf[rx_tail];
            rx_tail = (uint8_t)((rx_tail + 1u) % RX_BUF_SIZE);
        }
    }

    return data;
}

/* =========================
   SAIDAS
   ========================= */

static void traffic_off(void)
{
    PORTD &= (uint8_t)~((1 << LED_GREEN) |
                        (1 << LED_YELLOW) |
                        (1 << LED_RED));
}

static void show_state(void)
{
    traffic_off();

    if (emergency)
    {
        PORTD |= (1 << LED_RED);
        return;
    }

    if (state == STATE_GREEN)
    {
        PORTD |= (1 << LED_GREEN);
    }
    else if (state == STATE_YELLOW)
    {
        PORTD |= (1 << LED_YELLOW);
    }
    else
    {
        PORTD |= (1 << LED_RED);
    }
}

static void led_ped(uint8_t on)
{
    if (on)
    {
        PORTC |= (1 << LED_PED);
    }
    else
    {
        PORTC &= (uint8_t)~(1 << LED_PED);
    }
}

static void led_low_vis(uint8_t on)
{
    if (on)
    {
        PORTD |= (1 << LED_LOW_VIS);
    }
    else
    {
        PORTD &= (uint8_t)~(1 << LED_LOW_VIS);
    }
}

static void led_fault(uint8_t on)
{
    if (on)
    {
        PORTB |= (1 << LED_FAULT);
    }
    else
    {
        PORTB &= (uint8_t)~(1 << LED_FAULT);
    }
}

/* =========================
   GPIO
   ========================= */

static void io_init(void)
{
    DDRD |= (1 << LED_GREEN) |
            (1 << LED_YELLOW) |
            (1 << LED_RED) |
            (1 << LED_LOW_VIS);

    DDRB |= (1 << LED_FAULT) |
            (1 << LED_PWM);

    DDRC |= (1 << LED_PED);

    DDRD &= (uint8_t)~(1 << BTN_PED);
    PORTD |= (1 << BTN_PED);

    DDRC &= (uint8_t)~((1 << BTN_RESET_PED) |
                       (1 << BTN_LOW_VIS) |
                       (1 << BTN_FAULT));

    PORTC |= (1 << BTN_RESET_PED) |
             (1 << BTN_LOW_VIS) |
             (1 << BTN_FAULT);

    traffic_off();
    led_ped(0);
    led_low_vis(0);
    led_fault(0);
}

/* =========================
   TIMER1 PWM - PB1 / OC1A
   ========================= */

static void pwm_set(uint8_t duty)
{
    if (duty > 100)
    {
        duty = 100;
    }

    OCR1A = (uint16_t)((255UL * duty) / 100UL);
}

static void timer1_pwm_init(void)
{
    /*
     * Fast PWM 8-bit
     * OC1A nao invertido
     * prescaler = 64
     */
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

    pwm_set(50);
}

/* =========================
   TIMER2 CTC - 10 ms
   ========================= */

static void timer2_ctc_init(void)
{
    /*
     * 16 MHz / 1024 / (155 + 1) ~= 100 Hz
     * tick ~= 10 ms
     */
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);

    OCR2A = 155;
    TIMSK2 = (1 << OCIE2A);
}

/* =========================
   INT1 - PD3
   ========================= */

static void int1_init(void)
{
    EICRA |= (1 << ISC11);
    EICRA &= (uint8_t)~(1 << ISC10);

    EIMSK |= (1 << INT1);
}

/* =========================
   USART RX - 9600 bps
   ========================= */

static void usart0_rx_init(void)
{
    uint16_t ubrr = 103u;

    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr & 0xFFu);

    UCSR0A = 0;
    UCSR0B = (1 << RXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

/* =========================
   I2C / TWI MESTRE COM TIMEOUT
   ========================= */

static void twi_master_init(void)
{
    TWSR = 0x00;
    TWBR = 72;       /* aproximadamente 100 kHz */
    TWCR = (1 << TWEN);
}

static uint8_t twi_wait_timeout(void)
{
    uint16_t timeout = 60000u;

    while (!(TWCR & (1 << TWINT)))
    {
        timeout--;

        if (timeout == 0)
        {
            return 0;
        }
    }

    return 1;
}

static uint8_t twi_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

    if (!twi_wait_timeout())
    {
        return 0xFF;
    }

    return (uint8_t)(TWSR & 0xF8);
}

static uint8_t twi_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);

    if (!twi_wait_timeout())
    {
        return 0xFF;
    }

    return (uint8_t)(TWSR & 0xF8);
}

static void twi_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

static uint8_t twi_send_command(uint8_t addr, uint8_t cmd)
{
    uint8_t status;

    status = twi_start();

    if ((status != 0x08) && (status != 0x10))
    {
        twi_stop();
        return 0;
    }

    status = twi_write((uint8_t)(addr << 1));

    if (status != 0x18)
    {
        twi_stop();
        return 0;
    }

    status = twi_write(cmd);

    if (status != 0x28)
    {
        twi_stop();
        return 0;
    }

    twi_stop();
    return 1;
}

/* =========================
   LOGICA RECEBIDA DO MCU B
   ========================= */

static void apply_visibility(uint8_t value)
{
    if (value == VIS_DARK)
    {
        low_visibility = 1;
        led_low_vis(1);
        pwm_set(80);

        time_yellow = 200;
        time_red = 500;
    }
    else if (value == VIS_BRIGHT)
    {
        low_visibility = 0;
        led_low_vis(0);
        pwm_set(20);

        time_yellow = 120;
        time_red = 250;
    }
    else
    {
        low_visibility = 0;
        led_low_vis(0);
        pwm_set(50);

        time_yellow = 120;
        time_red = 300;
    }
}

static void apply_traffic(uint8_t value)
{
    if (value == TRAFFIC_HIGH)
    {
        time_green = 500;
    }
    else if (value == TRAFFIC_LOW)
    {
        time_green = 250;
    }
    else
    {
        time_green = 300;
    }
}

static void apply_emergency(uint8_t value)
{
    if (value == EMERGENCY_ON)
    {
        emergency = 1;
        led_fault(1);
        pwm_set(80);
    }
    else
    {
        emergency = 0;
        led_fault(0);

        if (!low_visibility)
        {
            pwm_set(50);
        }
    }

    show_state();
}

static void accept_ped(void)
{
    ped_request = 1;
    led_ped(1);
    time_red = 500;

    if (state == STATE_GREEN)
    {
        if (tick_state + 80 < time_green)
        {
            tick_state = (uint16_t)(time_green - 80);
        }
    }
}

static void process_frame(uint8_t type, uint8_t value)
{
    if (type == MSG_VISIBILITY)
    {
        apply_visibility(value);
    }
    else if (type == MSG_TRAFFIC)
    {
        apply_traffic(value);
    }
    else if (type == MSG_PED_RESULT)
    {
        if (value == PED_ACCEPTED)
        {
            accept_ped();
        }
    }
    else if (type == MSG_EMERGENCY)
    {
        apply_emergency(value);
    }
}

static void usart_process(void)
{
    static uint8_t frame_state = 0;
    static uint8_t type = 0;
    static uint8_t value = 0;
    uint8_t b;

    while (rx_available())
    {
        b = rx_get();

        if (frame_state == 0)
        {
            if (b == FRAME_SOF)
            {
                frame_state = 1;
            }
        }
        else if (frame_state == 1)
        {
            type = b;
            frame_state = 2;
        }
        else if (frame_state == 2)
        {
            value = b;
            frame_state = 3;
        }
        else
        {
            if (b == (uint8_t)(FRAME_SOF ^ type ^ value))
            {
                process_frame(type, value);
            }

            frame_state = 0;
        }
    }
}

/* =========================
   BOTOES LOCAIS
   ========================= */

static void local_buttons(void)
{
    if (!(PIND & (1 << BTN_PED)))
    {
        ped_request = 1;
        need_i2c_ped = 1;
        led_ped(1);
        accept_ped();
    }

    if (!(PINC & (1 << BTN_RESET_PED)))
    {
        ped_request = 0;
        led_ped(0);
    }

    if (!(PINC & (1 << BTN_LOW_VIS)))
    {
        apply_visibility(VIS_DARK);
    }

    if (!(PINC & (1 << BTN_FAULT)))
    {
        apply_emergency(EMERGENCY_ON);
    }
}

/* =========================
   INTERRUPCOES
   ========================= */

ISR(INT1_vect)
{
    ped_request = 1;
    need_i2c_ped = 1;
    led_ped(1);
    accept_ped();
}

ISR(USART_RX_vect)
{
    uint8_t data = UDR0;
    uint8_t next = (uint8_t)((rx_head + 1u) % RX_BUF_SIZE);

    if (next != rx_tail)
    {
        rx_buf[rx_head] = data;
        rx_head = next;
    }
}

ISR(TIMER2_COMPA_vect)
{
    uint16_t limit;

    tick_poll++;

    if (emergency)
    {
        show_state();
        return;
    }

    tick_state++;

    if (state == STATE_GREEN)
    {
        limit = time_green;
    }
    else if (state == STATE_YELLOW)
    {
        limit = time_yellow;
    }
    else
    {
        limit = time_red;
    }

    if (tick_state >= limit)
    {
        tick_state = 0;

        if (state == STATE_GREEN)
        {
            state = STATE_YELLOW;
        }
        else if (state == STATE_YELLOW)
        {
            state = STATE_RED;
        }
        else
        {
            state = STATE_GREEN;

            if (ped_request)
            {
                ped_request = 0;
                led_ped(0);
                time_red = 300;
            }
        }

        show_state();
    }
}

/* =========================
   MAIN
   ========================= */

int main(void)
{
    io_init();
    timer1_pwm_init();
    timer2_ctc_init();
    int1_init();
    usart0_rx_init();
    twi_master_init();

    show_state();

    sei();

    while (1)
    {
        uint8_t do_poll = 0;

        local_buttons();
        usart_process();

        if (need_i2c_ped)
        {
            need_i2c_ped = 0;
            (void)twi_send_command(MCU_B_ADDR, I2C_CMD_PED_REQUEST);
        }

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            if (tick_poll >= 100)
            {
                tick_poll = 0;
                do_poll = 1;
            }
        }

        if (do_poll)
        {
            (void)twi_send_command(MCU_B_ADDR, I2C_CMD_POLL);
        }
    }
}