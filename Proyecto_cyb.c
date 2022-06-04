/*
 * File:   Proyecto_cyb.c
 * Author: Carolina Paz
 *
 * Created on 3 de junio de 2022, 07:47 PM
 */


// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 8000000
#define FLAG_SPI 0xFF

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
char cont_master = 0;
char cont_slave = 0xFF;
char val_temporal = 0;
int CCPR = 0;
/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);
/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if (PIR1bits.SSPIF){
        val_temporal = SSPBUF; 
        PORTD=val_temporal;
        CCPR = map(val_temporal, 0, 255, 0, 1004); // Valor de ancho de pulso
        CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
        CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
        PIR1bits.SSPIF = 0;             // Limpiamos bandera de interrupción
  
    }
    
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){        
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0;
    ANSELH = 0;
    
    TRISB = 0;
    PORTB = 0;
    
    TRISD = 0;
    PORTD = 0;
    
    TRISA = 0b00100001;
    PORTA = 0;
    
    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Reloj interno
    
    // Configuración de SPI
    // Configs del esclavo
    TRISC = 0b00011000; // -> SDI y SCK entradas, SD0 como salida
    PORTC = 0;
        
    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0100;   // -> SPI Esclavo, SS hablitado
    SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
    SSPSTATbits.SMP = 0;        // -> Dato al final del pulso de reloj
     
        // Configuración PWM
    TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
    PR2 = 250;                  // periodo de 2ms
    
    // Configuración CCP
    CCP1CON = 0;                // Apagamos CCP1
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // PWM
    
    CCPR1L = 125>>2;
    CCP1CONbits.DC1B = 125 & 0b11;    // 0.5ms ancho de pulso / 25% ciclo de trabajo
    
    // CCP2 
    TRISCbits.TRISC1 = 1;       // Deshabilitamos salida de CCP2
    CCP2CON = 0;                // Apagamos CCP2
    CCP2CONbits.CCP2M = 0b1100; // PWM
    CCPR2L = 25>>2;              // Valor mínimo
    CCP2CONbits.DC2B0 = 25 & 0b1;
    CCP2CONbits.DC2B1 = 25>>1 & 0b1;
    
    
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM
    
    PIR1bits.SSPIF = 0;         // Limpiamos bandera de SPI
    PIE1bits.SSPIE = 1;         // Habilitamos int. de SPI
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}