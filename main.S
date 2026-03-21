; ****************************************************************************
; PROJETO: TRANSMISSOR BARE METAL 27MHz PARA ATMEGA2560
; FUNCIONALIDADE: Controle via Teclado (W,A,S,D), Telemetria e Fail-Safe
; ****************************************************************************

.include "m2560def.inc" ; Definições oficiais de registradores

; --- Mapeamento de Registradores ---
.def temp      = r16
.def status    = r17    ; Máquina de Estados: 0=IDLE, 1=PULSE, 2=SPACE
.def p_count   = r18    ; Contador de pulsos restantes
.def watchdog  = r19    ; Contador de Fail-Safe (Segurança)
.def b_low     = r20    ; Flag de bateria baixa

; --- Endereço de Memória para Comando ---
.equ CMD_ADDR  = 0x0200

; ****************************************************************************
; VETORES DE INTERRUPÇÃO
; ****************************************************************************
.org 0x0000
    rjmp reset          ; Reset do Microcontrolador
.org OC2Aaddr           ; Vetor do Timer 2 (Compare Match A)
    rjmp radio_engine_isr

; ****************************************************************************
; INICIALIZAÇÃO DO SISTEMA (SETUP)
; ****************************************************************************
reset:
    ; 1. Inicializar Stack Pointer (Pilha)
    ldi temp, low(RAMEND)
    out SPL, temp
    ldi temp, high(RAMEND)
    out SPH, temp

    ; 2. Configurar I/O (PB5/Pino 13 como saída)
    sbi DDRB, 5

    ; 3. Configurar UART0 (9600 bps @ 16MHz)
    ldi temp, 0
    sts UBRR0H, temp
    ldi temp, 103
    sts UBRR0L, temp
    ldi temp, (1 << RXEN0) | (1 << TXEN0)
    sts UCSR0B, temp
    ldi temp, (1 << UCSZ01) | (1 << UCSZ00)
    sts UCSR0C, temp

    ; 4. Configurar Timer 2 (Modo CTC para pulsos de 500us)
    ldi temp, (1 << WGM21)
    sts TCCR2A, temp
    ldi temp, 62        ; Valor para 500us @ 128 prescaler
    sts OCR2A, temp
    ldi temp, (1 << OCIE2A)
    sts TIMSK2, temp
    ldi temp, (1 << CS22) | (1 << CS20) ; Inicia com Prescaler 128
    sts TCCR2B, temp

    ; 5. Configurar ADC (Monitor de Bateria)
    ldi temp, (1 << REFS0) | (1 << ADLAR) ; AVCC e Justificado à Esquerda
    sts ADMUX, temp
    ldi temp, (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0)
    sts ADCSRA, temp

    clr status
    clr p_count
    clr watchdog
    sei                 ; Habilita Interrupções Globais

; ****************************************************************************
; LOOP PRINCIPAL (MAIN LOOP)
; ****************************************************************************
main_loop:
    rcall check_serial      ; Escuta o teclado do PC
    rcall monitor_battery   ; Verifica saúde da bateria
    rjmp main_loop

; ****************************************************************************
; SUB-ROTINAS
; ****************************************************************************

check_serial:
    lds temp, UCSR0A
    sbrs temp, RXC0         ; Chegou tecla?
    ret
    lds r21, UDR0           ; r21 recebe a tecla

    ; Tabela de Comandos (W, S, A, D)
    cpi r21, 'W'
    breq set_frente
    cpi r21, 'S'
    breq set_tras
    cpi r21, 'A'
    breq set_esq
    cpi r21, 'D'
    breq set_dir
    ret

set_frente:   ldi temp, 16 | rjmp dispatch
set_tras:     ldi temp, 40 | rjmp dispatch
set_esq:      ldi temp, 52 | rjmp dispatch
set_dir:      ldi temp, 64 | rjmp dispatch

dispatch:
    sts CMD_ADDR, temp      ; Envia comando para a ISR
    ldi watchdog, 200       ; Reset do Fail-Safe (~2 seg)
    ret

monitor_battery:
    sbi ADCSRA, ADSC        ; Inicia conversão ADC
wait_adc:
    sbic ADCSRA, ADSC
    rjmp wait_adc
    lds temp, ADCH          ; Lê apenas os 8 bits mais significativos
    cpi temp, 180           ; Limite de bateria (aprox 3.5V)
    brsh batt_ok
    ldi r21, 'B'            ; Alerta de Bateria Baixa
    rcall uart_tx
batt_ok:
    ret

uart_tx:
    lds temp, UCSR0A
    sbrs temp, UDRE0
    rjmp uart_tx
    sts UDR0, r21
    ret

; ****************************************************************************
; INTERRUPÇÃO DO MOTOR DE RÁDIO (ISR)
; ****************************************************************************
radio_engine_isr:
    push temp
    in temp, SREG
    push temp

    ; --- Fail-Safe ---
    tst watchdog
    breq safety_stop
    dec watchdog
    rjmp state_machine

safety_stop:
    clr p_count
    cbi PORTB, 5
    clr status
    rjmp isr_exit

state_machine:
    tst p_count
    brne process_p
    lds p_count, CMD_ADDR   ; Carrega novos pulsos da memória
    tst p_count
    breq isr_exit
    clr temp
    sts CMD_ADDR, temp      ; Limpa buffer de comando
    ldi status, 1

process_p:
    cpi status, 1
    breq do_pulse
    cbi PORTB, 5            ; Estado SPACE (Desligado)
    dec p_count
    ldi status, 1
    rjmp isr_exit
do_pulse:
    sbi PORTB, 5            ; Estado PULSE (Ligado)
    ldi status, 2

isr_exit:
    pop temp
    out SREG, temp
    pop temp
    reti
