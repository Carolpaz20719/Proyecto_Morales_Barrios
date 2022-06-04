/*
 * File:   PROYECTO2.c
 * Author: Cristy Morales
 *
 * Created on 18 de mayo de 2022, 10:49
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON     // Power-up Timer Enable bit (PWRT disabled)
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


#include <xc.h>
#include <stdint.h>
#include <stdio.h>

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 8000000 //Valor del reloj 

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
int speed; // velocidad de los DCs
int counter; // Contador de bit banging de los DCs
char servo1; // Control de servos
char servo2; 
char lec1;   //lecturas obtenidas
char lec2;
char direccion; //Dirección en la localidad de la EEPROM
char flag;  //Bandera de activación de Eusart
char dato;  //Dato a guardae en EEPROM


/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
char leer_EEPROM(char direccion);
void escribir_EEPROM(char dato, char direccion);
void putch(char data);
void comunicacion (void);


/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PIR1bits.ADIF == 1){ 
       
        if(ADCON0bits.CHS == 0){    // Verificamos sea AN0 el canal seleccionado, verificamos el canal que se esta convirtiendo
            servo1 = ADRESH;        //Dependiendo del canal guardamos el resultado 
            CCPR1L = (servo1 >> 1) + 125;   // Guardamos el resultado en CCP1
            CCP1CONbits.DC1B1 = (servo1 & 0b01);
            CCP1CONbits.DC1B0 = (servo1 >> 7);
        }
        
        else if(ADCON0bits.CHS == 1){    // Verificamos sea AN0 el canal seleccionado, verificamos el canal que se esta convirtiendo
            servo2 = ADRESH;        //Dependiendo del canal guardamos el resultado 
            CCPR2L = (servo2 >> 1) + 125;   // Guardamos el resultado en CCP1
            CCP2CONbits.DC2B1 = (servo2 & 0b01);
            CCP2CONbits.DC2B0 = (servo2 >> 7);
        }    
        
        else if (ADCON0bits.CHS == 2){
            speed = ADRESH;     //Guardamos el resultado en una variable para poder controlar la velocidad de los DC
            PORTD = speed;
        }
        PIR1bits.ADIF = 0; // Limpiamos bandera ADC (reiniciamos la interrupción)  
    } 
    
    if (INTCONbits.T0IF == 1){
        
        counter++;          //Creamos un contador que tenga de 0 a 255 un T= 1 seg
        INTCONbits.T0IF = 0;    //Limpiamos la bandera 
        TMR0 = 131;         //Reset timer 
        
        //banging de los DC
        if (counter >= speed){ //Comparamos el valor del ADC con el contador 
          //  PORTDbits.RD0 = 0;  //Si el contador es mayor los puertos son 0
           // PORTDbits.RD1 = 0;
        }
        
        else {
            //PORTDbits.RD0 = 1;  //Si es menos los puertos son 1
            //PORTDbits.RD1 = 1;   
        }
        
        if (counter == 256){ //Reiniciamos el contador 
            counter = 0;
        }
    } 
    
    if (INTCONbits.RBIF == 1){
        
        if  (PORTBbits.RB0 == 0){ //Guardado en memoria 
            PORTBbits.RB3 = 1;
            
            escribir_EEPROM(servo1, 0x10); //comenzamos el guardado
            escribir_EEPROM(servo2, 0x11);
            
            __delay_ms(1000);
  
        }
        
        else if (PORTBbits.RB1 == 0){ //lectura en la memoria 
            ADCON0bits.ADON = 0;    //Apagamos ADC para que no genere conflicto
            PORTBbits.RB4 = 1;
            
            lec1 = leer_EEPROM(0x10); //Leemos y guardamos los datos e la EEPROM Y luego al modulo PWM
            lec2 = leer_EEPROM(0x11);  
            
            CCPR1L = (lec1 >> 1) +125;
            CCPR2L = (lec2 >> 1) +125;
            
            __delay_ms(2500);
            ADCON0bits.ADON = 1;            //Renudamos operaciones 
            
        }
        
        else if (PORTBbits.RB2 == 0){
            if (flag == 0){         // Activamos la bandera que permitar usar EUSART
                PORTBbits.RB5 = 1;
                flag = 1;
            }
            
            else {
                PORTBbits.RB5 = 0;
                flag = 0;
                
            }    
        }
        
        else {
            PORTBbits.RB3 = 0;
            PORTBbits.RB4 = 0;
        }
        
        INTCONbits.RBIF = 0;
              
    }
}


/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/

void main(void) {
    setup();            //Llamamos la configuración del pic
    ADCON0bits.GO = 1;  //inicio a la conversión
            
            
    while(1)
    {
        SSPBUF = speed;   // Cargamos valor del contador al buffer
        while(!SSPSTATbits.BF){}// Esperamos a que termine el envio    
        __delay_ms(10);
        
    
        if(flag == 1){             // No hay proceso de conversion
            comunicacion();              // Iniciamos proceso de conversión
        }
        
        else { //si no esta activada la bandera funciona el ADC 
            if(ADCON0bits.GO == 0){           // Revisar cuando termine la conversion
                if(ADCON0bits.CHS == 0){       // verificar si el canal es 
                    ADCON0bits.CHS = 1;       // Cambio de canal indicado   
                }
                else if (ADCON0bits.CHS == 1){
                    ADCON0bits.CHS = 2;   
                }
                else if (ADCON0bits.CHS == 2){
                    ADCON0bits.CHS = 0;   
                }  
                
                __delay_us (200);
                ADCON0bits.GO = 1;
                
            }  
        }
    }
    
    return; 
}


/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0X07;     //entradas analogicas
    ANSELH = 0X00;         // I/O digitales)
    
    
    TRISA = 0X07; // entradas y salidas 
    TRISB = 0X07;
    TRISD = 0X00;
    
    OPTION_REGbits.nRBPU = 0; //activamos pull up puerto b
    WPUB = 0x07;
    IOCB = 0x07;
    
    PORTA = 0x00; 
    PORTB = 0x00;
    PORTD = 0x00;
    
    // Configuración reloj interno
    OSCCONbits.IRCF = 0b111;    // 8MHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // Configuracion interrupciones
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    INTCONbits.T0IE = 1;        // Habilitamos interrupcion TMR0
    INTCONbits.RBIE = 1;
    
                                //limpiamos banderas
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC    
    INTCONbits.T0IF = 0;        // Limpiamos bandera de interrupción TMR0
    INTCONbits.RBIF = 1;
    
    //Configuración TMR0
    //Timer0 Registers Prescaler= 64 - TMR0 Preset = 131 - Freq = 250.00 Hz - Period = 0.004000 seconds
    OPTION_REGbits.T0CS = 0;  // bit 5  TMR0 Clock Source Select bit...0 = Internal Clock (CLKO) 1 = Transition on T0CKI pin
    OPTION_REGbits.T0SE = 0;  // bit 4 TMR0 Source Edge Select bit 0 = low/high 1 = high/low
    OPTION_REGbits.PSA = 0;   // bit 3  Prescaler Assignment bit...0 = Prescaler is assigned to the Timer0
    OPTION_REGbits.PS2 = 1;   // bits 2-0  PS2:PS0: Prescaler Rate Select bits
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS0 = 1;
    TMR0 = 131;             // preset for timer register

    // Configuración ADC
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    
    ADCON0bits.ADCS = 0b10;     // Fosc/32
    ADCON0bits.CHS = 0;         // Seleccionamos el AN0
    __delay_us(200);             // Sample time    
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(200);             // Sample time
    
    //Configuración PMW
    TRISCbits.TRISC2 = 1 ;  
    TRISCbits.TRISC1 = 1 ;      //RC1 y RC2 como entrada
    PR2 = 250;                  //Config del periodo
    
    //configuración CCP
    CCP1CONbits.P1M = 0;
    CCP1CONbits.CCP1M = 0b1100;  //Iniciamos el modo PMW
    CCP2CONbits.CCP2M = 0b1100; 
    
    CCPR1L = 0x0f;          //ciclo de trabajo incial
    CCPR2L = 0x0f;
    CCP1CONbits.DC1B = 0;   //bits menos significativos 
    CCP2CONbits.DC2B0 = 0; 
    CCP2CONbits.DC2B1 = 0; 
    
    //Configuraciones TMR2
    PIR1bits.TMR2IF = 0;        //limpiamos bandera
    T2CONbits.T2CKPS = 0b11;    //preescalar de 16
    T2CONbits.TMR2ON = 1;       //encendemos el TMR2
    
    while(PIR1bits.TMR2IF == 0);   //Esperamos a que se de una interrupcion
    PIR1bits.TMR2IF = 0;        //limpiamos la bandera
    TRISCbits.TRISC2 = 0 ;         //RC1 y RC2 como salidas 
    TRISCbits.TRISC1 = 0 ;    
    
    //CONFIGURACIÓN SERIAL COMUNICACION 
    TXSTAbits.SYNC = 0;         //Modo asincronico
    TXSTAbits.BRGH = 1;         //Activamos la alta velocidad del Baud rate
    BAUDCTLbits.BRG16 = 1;      //utilizamos los 16 bits de baud
    
    //VELOCIDAD DE BAU RATES, baud rate de 9600
    SPBRGH = 0;
    SPBRG = 207; //BYTE INFERIOR 

    //ACTIVAR MODULO SERIAL
    RCSTAbits.SPEN = 1; //Activamos los puertos seriales
    RCSTAbits.RX9 = 0;  //No utilizamos los 9 bits
    RCSTAbits.CREN = 1; //recepcion continua
    TXSTAbits.TXEN = 1; //transmisión
    
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC4 = 1;
    TRISCbits.TRISC5 = 0;

    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0000;   // -> SPI Maestro, Reloj -> Fosc/4 (250kbits/s)
    SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
    SSPSTATbits.SMP = 1;        // -> Dato al final del pulso de reloj
    SSPBUF = speed;            // Enviamos un dato inicial
    return;
    
}


/*------------------------------------------------------------------------------
 * Memoria EEPROM
 ------------------------------------------------------------------------------*/
char leer_EEPROM(char direccion){
    EEADR = direccion;
    
    EECON1bits.EEPGD = 0;       // Lectura a la EEPROM
    EECON1bits.RD = 1;          // Obtenemos dato de la EEPROM
    
    char dato = EEDATA;
    return dato;               // Regresamos dato 
}

void escribir_EEPROM (char dato, char direccion){
    EEADR = direccion;
    EEDAT = dato;
    EECON1bits.EEPGD = 0;       // Escritura a la EEPROM
    EECON1bits.WREN = 1;        // Habilitamos escritura en la EEPROM
    
    INTCONbits.GIE = 0;         // Deshabilitamos interrupciones
    EECON2 = 0x55;      
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;          // Iniciamos escritura
    
    while(PIR2bits.EEIF == 0); //esperamos que termine la escritura
    PIR2bits.EEIF = 0;          // limpiamos bandera
    EECON1bits.WREN = 0;        // Deshabilitamos escritura en la EEPROM

}

/*------------------------------------------------------------------------------
 * EUSART
 ------------------------------------------------------------------------------*/

void putch(char data){
    while (TXIF == 0);
    TXREG = data;
    return;
}

void comunicacion (void){
    
    __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
    printf("\r Que desea realizar? \r");
 
    __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
    printf(" 1) controlar motores \r");
    
    __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
    printf(" 2) EEPROM \r");   
    
    __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
    printf(" 3) Finalizar \r");   
    
    while (RCIF == 0); //esperar a que se ingrese un dato
    
    if (RCREG == '1'){ //Si presionamos el 1 
        printf("\r\r Que motor desea controlar? \r");   
        
        __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
        printf(" 1) servomotor 1 \r");

        __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
        printf(" 2) servomotor 2 \r");   

        __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
        printf(" 3) motores DC \r");  
        
        while (RCIF == 0); //esperar a que se ingrese un dato

        if (RCREG == '1'){ //Si presionamos el 1 
            printf("\r\r Que direccion? \r");   

            __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
            printf(" a) Derecha \r");

            __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
            printf(" b) Izquierda \r");   

            __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
            printf(" c) Centro \r");  
            
            while (RCIF == 0); //esperar a que se ingrese un dato
            
            if (RCREG == 'a'){
                CCPR1L = (250 >> 1)+125;
            }
            
            else if (RCREG == 'b'){
                CCPR1L = (0 >> 1)+125;
            }
            
            else if (RCREG == 'c'){
                CCPR1L = (127 >> 1)+125;
            }
            
            else {
                NULL;
            }
                                   
        }   
        
        
        if (RCREG == '2'){ //Si presionamos el 1 
            printf("\r\r Que direccion? \r");   

            __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
            printf(" a) Derecha \r");

            __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
            printf(" b) Izquierda \r");   

            __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
            printf(" c) Centro \r");  
            
            while (RCIF == 0); //esperar a que se ingrese un dato
            
            if (RCREG == 'a'){
                CCPR2L = (250 >> 1)+125;
            }
            
            else if (RCREG == 'b'){
                CCPR2L = (0 >> 1)+125;
            }
            
            else if (RCREG == 'c'){
                CCPR2L = (127 >> 1)+125;
            }
            
            else {
                NULL;
            }
                                   
        }   
        
        if (RCREG == '3'){ //Si presionamos el 1 
            printf("\r\r Que velocidad? \r");   

            __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
            printf(" a) Rapido \r");

            __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
            printf(" b) Freno \r");   

            __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
            printf(" c) Moderado \r");  
            
            while (RCIF == 0); //esperar a que se ingrese un dato
            
            if (RCREG == 'a'){
                speed = 255;
            }
            
            else if (RCREG == 'b'){
                speed = 0;
            }
            
            else if (RCREG == 'c'){
                speed = 127;
            }
            
            else {
                NULL;
            }
                                   
        }                  
    
    }
    
    
    else if (RCREG == '2'){
        printf("\r\r Que desea hacer en la memoria EEPROM? \r");   
        
        __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
        printf(" a) Grabar \r");

        __delay_ms (100); //Printf llama la función putch para enviar a todos los caracteres dentro de las comillas
        printf(" b) leer \r");   

        while (RCIF == 0); //esperar a que se ingrese un dato   
        
        if (RCREG == 'a'){
            PORTBbits.RB3 = 1;
            escribir_EEPROM (servo1, 0x10);
            escribir_EEPROM (servo2, 0x11);
            
            
            __delay_ms (1000);
            PORTBbits.RB3 = 0;
        }
        
        else if (RCREG == 'b'){
            ADCON0bits.ADON = 0;
            PORTBbits.RB4 = 1;
            
            lec1 = leer_EEPROM (0x10);
            lec2 = leer_EEPROM (0x11);
            
            CCPR1L = (lec1 >> 1) +125;
            CCPR2L = (lec2 >> 1) +125;
            
            __delay_ms(2500);
            ADCON0bits.ADON = 1;
            PORTBbits.RB4 = 0;
     
        }
        
        else {
            NULL;
        }
  
    }
    
    else if (RCREG == '3'){
        __delay_ms(500);
        printf("\r\r CHAUU \r ");
        flag = 0;
        PORTBbits.RB5 = 0;
    }
    
    else {
        NULL;
    }
    
    return;
    
}