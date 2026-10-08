# Semáforos com passadeira (2 × ATmega328P)

Trabalho prático individual de Microprocessadores e Microcontroladores, Licenciatura em Engenharia Eletrónica e de Telecomunicações, Universidade Autónoma de Lisboa (junho de 2026).

Sistema de semáforos com passadeira para peões, com dois microcontroladores ATmega328P a 16 MHz, programados em C (Microchip Studio / AVR-GCC) e simulados no SimulIDE 1.1.0.

## Arquitetura

**MCU A: gestor de sinalização** (`main_gestores.c`)
- Máquina de estados verde, amarelo e vermelho, temporizada pelo Timer2 em modo CTC (tick de 10 ms).
- Brilho dos LEDs por PWM (Timer1, Fast PWM de 8 bits, PB1).
- Pedido de passagem do peão por interrupção externa INT1 (PD3).
- Receção USART a 9600 bps com buffer circular de 32 bytes e validação das tramas.
- I2C/TWI mestre (cerca de 100 kHz) com timeout.
- Botões locais: reset do pedido de peão, baixa visibilidade e falha/emergência.

**MCU B: unidade de sensores** (`main_sensores.c`)
- ADC de 10 bits: ADC0 (luminosidade), ADC1 (tráfego) e ADC2 (falha), simulados com potenciómetros.
- Deteção de intrusão por interrupção de mudança de pino (PCINT0, PB0).
- Envio dos estados ao MCU A por USART, a cada 500 ms.

## Comportamento adaptativo
- Baixa luminosidade: acende o LED de baixa visibilidade, aumenta o brilho (PWM a 80%) e alonga o amarelo (2 s) e o vermelho (5 s).
- Tráfego intenso: verde de 5 s. Tráfego baixo: verde de 2,5 s.
- Pedido de peão: o verde é encurtado e o vermelho passa a 5 s.
- Falha ou intrusão: o semáforo fica em vermelho fixo e acende o LED de emergência.

## Protocolo USART
Trama de 4 bytes: `0xA5` (início), tipo, valor e checksum (XOR dos três anteriores).
Tipos: `0x10` visibilidade, `0x11` tráfego, `0x13` emergência.

## Ficheiros
- `main_gestores.c` e `main_sensores.c`: código-fonte dos dois microcontroladores.
- `GccApplicationgestores.hex` e `GccApplicationsensores.hex`: programas compilados.
- `MCA_simulide.sim1`: circuito para o SimulIDE 1.1.0.
- `Relatório Final de microprocessadores e microcontroladores.pdf`: relatório do trabalho.

## Como executar
Abra `MCA_simulide.sim1` no SimulIDE 1.1.0 e inicie a simulação. Cada ATmega328P carrega o `.hex` correspondente.

## Limitações e trabalho futuro
- O I2C está implementado só no lado mestre (MCU A). O MCU B ainda não responde como escravo.
- Mecanismos de confirmação de receção e repetição de mensagens na comunicação.
- Sinal sonoro ou contador visual para o peão.
