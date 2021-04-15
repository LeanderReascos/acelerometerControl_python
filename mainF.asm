#include <pic18f47q10.inc>
#include <xc.inc>
    
CONFIG FEXTOSC=0b100	;deactivate external oscillator (to allow write to RA7)
CONFIG CSWEN=0b1	;allow editing NDIV and NOSC for CLK config    
CONFIG WDTE=OFF		;required to avoid WDT restarting micro all the time

#define BAUD_RATE 11 ;baud rate
    
#define AXIS	    0 ; 0b00000011 = x, 0b00000010 = y, 0b00000001 = z
#define CHAR_MSB    1   
#define CHAR_LSB    2
#define CHAR	    3
#define AUX	    4


; CHAR - AXIS[7:6], MSB/LSB[5], DATA[4:0]
    
; DEFINIR axis = 3 ANTES DO LOOP.
    
PSECT code
ORG 0x0000
    goto start  
 
ORG 0x0008
    goto interrupts    
    
ORG 0x0030  
start:
    
    MOVLW   0X03
    MOVWF   AXIS
    /*****************
    CONFIGURACAO PORTA
    ******************/
    BANKSEL LATA
    CLRF    LATA
    MOVLW   0X0F
    BANKSEL TRISA
    MOVWF   TRISA
    BANKSEL ANSELA
    MOVWF   ANSELA
    
    /*****************
    CONFIGURACAO CLOCK
    ******************/
    BANKSEL OSCFRQ
    MOVLW   0X03 ;HFINTOSC = 8MHz ---- FOSC 
    MOVWF   OSCFRQ,1  
    BANKSEL OSCCON1   
    MOVLW   0X60 ;New Oscilator: OSCILATOR = HFINTOSC --- FOSC
    MOVWF   OSCCON1,1   
    BANKSEL OSCEN
    MOVLW   0b01000000 ;internal clock @freq=OSCFRQ ativo	
    MOVWF   OSCEN,1
    
    /******************
    CONFIGURACAO TIMER0
    *******************/
       /*TIMER_OSCILATOR = OSCILATOR/PRESCALER = 8MHz/2048 = 3906.5 Hz
	 com o PostScaler 1:5
	               TIMER_OSCILATOR = OSCILATOR/PRESCALER/POSTSCALER = 781.25 Hz
	 com o Comparador a 1
		       TIMER_INTERRUPT = Famostragem = TIMER_OSCILATOR/2 = 390.625 Hz*/
    BANKSEL T0CON0
    MOVLW   0x04  ;1:5 postscaler, 8bit
    MOVWF   T0CON0,1
    BANKSEL T0CON1
    MOVLW   0b01101011  ;Prescaler 1:2048
    MOVWF   T0CON1,1
    BANKSEL TMR0L   
    CLRF    TMR0L  ;clear the timer's lowest 8 bits (the counter)
    BANKSEL TMR0H
    MOVLW   0x01   ;set the timer's highest 8 bits (the comparator)
    MOVWF   TMR0H
    
    /***************
    CONFIGURACAO ADC
    ****************/
    BANKSEL ADPCH
    MOVLW   0X03
    MOVWF   ADPCH
    BANKSEL ADREF
    CLRF    ADREF
    BANKSEL ADCLK
    MOVLW   0x0F    ;FOSC/32 === 250KHz 
    MOVWF   ADCLK,1
    BANKSEL ADCON0
    BSF	    ADCON0,7  
    
    /*****************
    CONFIGURACAO PORTB
    ******************/
	;PORTB OUTPUT DIGITAL, RB4 INPUT DIGITAL
    BANKSEL LATB
    CLRF    LATB
    BANKSEL TRISB
    CLRF    TRISB
    BSF	    TRISB,4
    BANKSEL ANSELB
    CLRF    ANSELB
	;RB5 OUTPUT DIGITAL TX/CK 
    BANKSEL RB6PPS
    MOVLW   0x09  
    MOVWF   RB6PPS  
	;LIGAR RB4 A PERIPHERICAL INTERRUPT
    BANKSEL INT0PPS
    MOVLW   0x0C	  
    MOVWF   INT0PPS 
    
    
    /************************
    CONFIGURACAO INTERRUPCOES
    *************************/
    BANKSEL PIR1
    BCF	    PIR1,0  ;clear ADC interrupt flag
    BANKSEL PIR0
    BCF	    PIR0, 5 ;clear timer interrupt flag
    BCF	    PIR0,0   ;clear INT0 interrupt flag
    BANKSEL PIE0
    BSF	    PIE0, 5
    BSF	    PIE0,0   ; enable INT0
    BANKSEL PIE1
    BSF	    PIE1,0  ;enable adc int
    BANKSEL INTCON
    BSF	    INTCON,7
    BSF	    INTCON,6  
    
    
    
    /************************
    CONFIGURACAO SERIAL PORT
    *************************/

    MOVLW   BAUD_RATE	    ; Move BAUD_RATE to Baud Rate Generator -> expected a baud of 
    BANKSEL SP1BRGL
    MOVWF   SP1BRGL
    MOVLW   0x00
    MOVWF   SP1BRGH	; "1" for USART 1, since we have 2 USART available
    MOVLW   0b10100000	; 8 data bits, TX enabled, master clock selected
    BANKSEL TX1STA
    MOVWF   TX1STA	; Low speed SPBRG mode
    MOVLW   0b10000000	; USART enabled, 8 data bits / enable receiver 0b10010000
    BANKSEL RC1STA
    MOVWF   RC1STA ; Receiver enabled
    
main:
    BTG	    LATA,4
    GOTO    main
    
interrupts:
    BANKSEL PIR0
    BTFSC   PIR0, 5 
    goto    TIMER0_INT
    BANKSEL PIR1
    BTFSC   PIR1,0
    goto    ADC_Finish_Interrupt
    BTFSC   PIR0, 0
    goto    BUTTON_0
    RETFIE

    
TIMER0_INT:
    BANKSEL	PIR0
    BCF		PIR0	,5	; Limpar flag de interrupt do TIMER0
    BANKSEL	ADCON0
    BSF		ADCON0	,0	; Começar nova conversão do ADC   
    BTG		LATA, 6		; "Piscar" LED.
    RETFIE

ADC_Finish_Interrupt:
    MOVLW	0b01000000
    MULWF	AXIS		; Shift Left 6 casas
    MOVFF	PRODL, AUX	; Mover AXIS para variável auxiliar
    
    call	CHAR_DATA_MSB
    call	CHAR_DATA_LSB
    BTG		LATA, 7		; "Piscar" LED.
    DCFSNZ	AXIS		; Decrementar, skip se != 0.
    call	AXIS_RESET	; Voltar a definir AXIS = 0b00000011

    BANKSEL	ADPCH
    BTFSC	0X00,1
    BSF		ADPCH,1
    BTFSC	0X00,0
    BSF		ADPCH,0
    BTFSS	0X00,1
    BCF		ADPCH,1
    BTFSS	0X00,0
    BCF		ADPCH,0

    BANKSEL	PIR1
    BCF		PIR1,0		; Limpar flag do ADC
    ; ADC em standby
    
    RETFIE
    
AXIS_RESET:
    MOVLW	0b00000011
    MOVWF	AXIS
    return
    
CHAR_DATA_MSB:
    BANKSEL	ADRES
    MOVFF	ADRESH, CHAR_MSB
    
    MOVLW	0b00100000
    MULWF	CHAR_MSB
    MOVFF	PRODH, CHAR_MSB	; 5 MSB em [4:0]
    BSF		CHAR_MSB, 5	; Set bit "MSB"
    MOVFF	CHAR_MSB, CHAR
    
    MOVF	AUX , 0		; Colocar AXIS no WREG
    ADDWF	CHAR, 1		; Somar os bits de AXIS a CHAR, guardar em CHAR
    
    call	SENDCHAR	; Enviar CHAR
    
    return
    

CHAR_DATA_LSB:
    BANKSEL	ADRES
    MOVFF	ADRESH, CHAR_MSB
    BANKSEL	ADRES
    MOVFF	ADRESL, CHAR_LSB
    
    MOVLW	0b00000100
    MULWF	CHAR_MSB
    MOVFF	PRODL, CHAR_MSB	; Guardar temporariamente 3 LSB de ADRESH em [4:2]
    
    MOVLW	0b00000100
    MULWF	CHAR_LSB
    MOVFF	PRODH, CHAR_LSB	; 2 LSB de ADRES em [1:0]
    
    MOVF	CHAR_MSB, 0	; Colocar no WREG
    ADDWF	CHAR_LSB, 1	; CHAR_MSB + CHAR_LSB, guardado em CHAR_LSB - 5 LSB em [4:0]
    BCF		CHAR_LSB, 5	; Clear bit "MSB"
    BCF		CHAR_LSB, 6	; Limpar bit 6
    BCF		CHAR_LSB, 7	; Limpar bit 7
    MOVFF	CHAR_LSB, CHAR
    
    MOVF	AUX , 0		; Colocar AXIS no WREG
    ADDWF	CHAR, 1		; Somar os bits de AXIS a CHAR, guardar em CHAR
    
    call	SENDCHAR	; Enviar CHAR
    
    return
    

BUTTON_0:			; Ligar/Desligar comunicação
    BANKSEL	LATA
    BSF		LATA    ,6
    BTG		T0CON0	,7	; Toggle TIMER0 (Ligar/Desligar comunicação)
    MOVLW	0X20		; AXIS: 00, MSB: 1, DATA: 00000 - (Ligar + Calibrar) ou Desligar
    MOVWF	CHAR
    call	SENDCHAR
    MOVLW	0X01		; AXIS: 00, MSB: 0, DATA: 00001 - (Ligar + Calibrar) ou Desligar
    MOVWF	CHAR
    call	SENDCHAR
    BANKSEL	LATA
    BTG		LATA	,5	; "Piscar" LED
    BCF		LATA    ,6
    BCF		LATA    ,7
    BANKSEL	PIR0
    BCF		PIR0, 0		; Limpar INT0 interrupt test bit
    
    RETFIE


SENDCHAR:
    BANKSEL	PIR3
    btfss	PIR3,   4	; Ver se TX Buffer está disponível (TXIF)
    bra		SENDCHAR	; Se não, branch para tentar de novo
    BANKSEL	TX1REG
    movff	CHAR,   TX1REG	; Colocar CHAR no USART TX register para envio
    BANKSEL	LATA
    
    return
    
    
END


