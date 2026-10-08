/*
 * Trabalho 6 - Semáforos com passadeira
 * MCU B - Unidade de Sensores e Lógica de Via
 *
 * Versão funcional para SimulIDE:
 * - Lê ADC0, ADC1 e ADC2
 * - Usa PCINT0 em PB0 para intrusão
 * - Envia frames por USART em PD1/TX
 * - Mostra estado em LEDs de debug D4, D5, D6 e D7
 *
 * Target: ATmega328P
 * Clock: 16 MHz
 * Toolchain: Microchip Studio / AVR-GCC
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

/* =========================
   PROTOCOLO USART
   ========================= */

#define FRAME_SOF          0xA5

#define MSG_VISIBILITY     0x10
#define MSG_TRAFFIC        0x11
#define MSG_EMERGENCY      0x13

#define VIS_DARK           0u
#define VIS_NORMAL         1u
#define VIS_BRIGHT         2u

#define TRAFFIC_LOW        0u
#define TRAFFIC_MEDIUM     1u
#define TRAFFIC_HIGH       2u

#define EMERGENCY_OFF      0u
#define EMERGENCY_ON       1u

/* =========================
   PINOS MCU B
   ========================= */

/*
 * C0 / ADC0 = sensor de luz
 * C1 / ADC1 = sensor de tráfego
 * C2 / ADC2 = sensor de falha analógica
 *
 * B0 / PB0  = botão intrusão / PCINT0
 *
 * D1 / PD1  = TX USART para RX do MCU A
 *
 * LEDs de debug:
 * D4 = heartbeat, pisca para mostrar que MCU B está vivo
 * D5 = baixa visibilidade
 * D6 = tráfego alto
 * D7 = emergência
 */

#define PIN_INTRUSAO       PB0

#define LED_HEARTBEAT      PD4
#define LED_BAIXA_VIS      PD5
#define LED_TRAFEGO_ALTO   PD6
#define LED_EMERGENCIA     PD7

/* =========================
   VARIÁVEIS
   ========================= */

volatile uint8_t intrusao_ativa = 0;

/* =========================
   USART TX 9600 bps
   ========================= */

static void usart0_tx_init(void)
{
    /*
     * Baud rate = 9600 bps
     * F_CPU = 16 MHz
     * UBRR = 103
     */
    uint16_t ubrr = 103u;

    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr & 0xFFu);

    UCSR0A = 0;

    /* Ativa transmissor */
    UCSR0B = (1 << TXEN0);

    /* 8 bits, 1 stop bit, sem paridade */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void usart_send_byte(uint8_t data)
{
    while (!(UCSR0A & (1 << UDRE0)))
    {
    }

    UDR0 = data;
}

static void usart_send_frame(uint8_t type, uint8_t value)
{
    uint8_t checksum;

    checksum = (uint8_t)(FRAME_SOF ^ type ^ value);

    usart_send_byte(FRAME_SOF);
    usart_send_byte(type);
    usart_send_byte(value);
    usart_send_byte(checksum);
}

/* =========================
   ADC
   ========================= */

static void adc_init(void)
{
    /*
     * Referência AVCC
     * Resultado alinhado à direita
     * Prescaler = 128
     *
     * Frequência ADC = 16 MHz / 128 = 125 kHz
     */
    ADMUX = (1 << REFS0);

    ADCSRA = (1 << ADEN)  |
             (1 << ADPS2) |
             (1 << ADPS1) |
             (1 << ADPS0);

    /*
     * Desativa buffers digitais nos ADC0, ADC1 e ADC2.
     */
    DIDR0 = (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D);
}

static uint16_t adc_read(uint8_t channel)
{
    channel &= 0x07u;

    /*
     * Mantém referência AVCC e escolhe canal.
     */
    ADMUX = (uint8_t)((ADMUX & 0xF0u) | channel);

    ADCSRA |= (1 << ADSC);

    while (ADCSRA & (1 << ADSC))
    {
    }

    return ADC;
}

/* =========================
   PCINT0 - INTRUSÃO
   ========================= */

static void pcint0_init(void)
{
    /*
     * PB0 como entrada com pull-up interno.
     * Botão liga PB0 ao GND.
     */
    DDRB &= (uint8_t)~(1 << PIN_INTRUSAO);
    PORTB |= (1 << PIN_INTRUSAO);

    /*
     * Ativa interrupção por mudança no grupo PCINT0.
     */
    PCICR |= (1 << PCIE0);

    /*
     * Ativa PCINT0, que corresponde a PB0.
     */
    PCMSK0 |= (1 << PCINT0);

    intrusao_ativa = ((PINB & (1 << PIN_INTRUSAO)) == 0);
}

/* =========================
   GPIO DEBUG
   ========================= */

static void io_init(void)
{
    DDRD |= (1 << LED_HEARTBEAT) |
            (1 << LED_BAIXA_VIS) |
            (1 << LED_TRAFEGO_ALTO) |
            (1 << LED_EMERGENCIA);

    PORTD &= (uint8_t)~((1 << LED_HEARTBEAT) |
                        (1 << LED_BAIXA_VIS) |
                        (1 << LED_TRAFEGO_ALTO) |
                        (1 << LED_EMERGENCIA));
}

/* =========================
   CLASSIFICAÇÕES
   ========================= */

static uint8_t classificar_visibilidade(uint16_t adc_luz)
{
    if (adc_luz < 350u)
    {
        return VIS_DARK;
    }

    if (adc_luz > 750u)
    {
        return VIS_BRIGHT;
    }

    return VIS_NORMAL;
}

static uint8_t classificar_trafego(uint16_t adc_trafego)
{
    if (adc_trafego < 350u)
    {
        return TRAFFIC_LOW;
    }

    if (adc_trafego > 700u)
    {
        return TRAFFIC_HIGH;
    }

    return TRAFFIC_MEDIUM;
}

/* =========================
   INTERRUPÇÃO PCINT0
   ========================= */

ISR(PCINT0_vect)
{
    intrusao_ativa = ((PINB & (1 << PIN_INTRUSAO)) == 0);
}

/* =========================
   MAIN
   ========================= */

int main(void)
{
    uint16_t adc_luz;
    uint16_t adc_trafego;
    uint16_t adc_falha;

    uint8_t visibilidade;
    uint8_t trafego;
    uint8_t emergencia;

    io_init();
    adc_init();
    usart0_tx_init();
    pcint0_init();

    sei();

    while (1)
    {
        /*
         * Pisca LED D4 para mostrar que o MCU B está a correr.
         */
        PORTD ^= (1 << LED_HEARTBEAT);

        /*
         * Ler sensores.
         */
        adc_luz = adc_read(0);       /* C0 / ADC0 */
        adc_trafego = adc_read(1);   /* C1 / ADC1 */
        adc_falha = adc_read(2);     /* C2 / ADC2 */

        visibilidade = classificar_visibilidade(adc_luz);
        trafego = classificar_trafego(adc_trafego);

        /*
         * Emergência se:
         * - botão de intrusão ativo
         * - ADC2 muito alto
         */
        if (intrusao_ativa || (adc_falha > 950u))
        {
            emergencia = EMERGENCY_ON;
        }
        else
        {
            emergencia = EMERGENCY_OFF;
        }

        /*
         * LED D5: baixa visibilidade.
         */
        if (visibilidade == VIS_DARK)
        {
            PORTD |= (1 << LED_BAIXA_VIS);
        }
        else
        {
            PORTD &= (uint8_t)~(1 << LED_BAIXA_VIS);
        }

        /*
         * LED D6: tráfego alto.
         */
        if (trafego == TRAFFIC_HIGH)
        {
            PORTD |= (1 << LED_TRAFEGO_ALTO);
        }
        else
        {
            PORTD &= (uint8_t)~(1 << LED_TRAFEGO_ALTO);
        }

        /*
         * LED D7: emergência/falha.
         */
        if (emergencia == EMERGENCY_ON)
        {
            PORTD |= (1 << LED_EMERGENCIA);
        }
        else
        {
            PORTD &= (uint8_t)~(1 << LED_EMERGENCIA);
        }

        /*
         * Enviar informação para o MCU A por USART.
         * Frame: 0xA5, tipo, valor, checksum XOR.
         */
        usart_send_frame(MSG_VISIBILITY, visibilidade);
        usart_send_frame(MSG_TRAFFIC, trafego);
        usart_send_frame(MSG_EMERGENCY, emergencia);

        _delay_ms(500);
    }
}