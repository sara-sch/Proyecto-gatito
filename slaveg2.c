/* 
 * File:   slaveg.c
 * Author: saras
 *
 * Created on May 16, 2022, 4:06 PM
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
#define _XTAL_FREQ 1000000
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 255              // Valor máximo de entrada del potenciometro
#define OUT_MIN 16               // Valor minimo de ancho de pulso de señal PWM
#define OUT_MAX 80             // Valor máximo de ancho de pulso de señal PWM
/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
char cont_master = 0;
char cont_slave = 0xFF;
char cont = 0;
char cont2 = 0;
unsigned short CCPR = 0;        // Variable para almacenar ancho de pulso al hacer la interpolación lineal
/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);

void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);
/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if (PIR1bits.SSPIF){
        CCPR = map(SSPBUF, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso variable
        CCPR1L = (uint8_t)(CCPR>>2);    // Se guardan los 8 bits más significativos en CPR1L
        CCP1CONbits.DC1B = CCPR & 0b11; // Se guardan los 2 bits menos significativos en DC1B
        
        PIR1bits.SSPIF = 0;  // Limpiamos bandera de interrupción
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
    
    TRISA = 0b00100001;
    PORTA = 0;
    
    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Reloj interno
    
    // Configuracion de SPI
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

    PIR1bits.SSPIF = 0;         // Limpiamos bandera de SPI
    PIE1bits.SSPIE = 1;         // Habilitamos int. de SPI
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    
    //Config PWM
    CCP1CON = 0; // Se apaga CCP1
    TRISCbits.TRISC2 = 1; // RC2/CCP1 como salida deshabilitado
    PR2 = 255; // Período de 16 ms 
    
    // Config CCP
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // Modo PWM
    
    CCPR1L = 61; //Ciclo de trabajo base pues se va a variar
    CCP1CONbits.DC1B = 61 & 0b11; // Base de 1 ms ancho de pulso
    
    PIR1bits.TMR2IF = 0;        // Limpieza de bandera del TMR2
    T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Se enciende TMR2
    while(!PIR1bits.TMR2IF);    // Se espera un ciclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpieza de bandera del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Se habilita salida de PWM

}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}