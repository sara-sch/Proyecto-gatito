/* 
 * File:   masterg.c
 * Author: saras
 *
 * Created on May 16, 2022, 3:38 PM
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
#pragma config BOR4V = BOR40V 
// Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
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
#define IN_MAX 255              // Valor m�ximo de entrada del potenciometro
#define OUT_MIN 16               // Valor minimo de ancho de pulso de se�al PWM
#define OUT_MAX 80             // Valor m�ximo de ancho de pulso de se�al PWM
/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
char pot3 = 0;
char pot4 = 0;
char cont_master = 0;
unsigned short CCPR = 0;        // Variable para almacenar ancho de pulso al hacer la interpolaci�n lineal
unsigned short CCPRB = 0;
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
    
    if(PIR1bits.ADIF){              // Fue interrupci�n del ADC?
            if(ADCON0bits.CHS == 1){
                CCPR = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
        }
            if(ADCON0bits.CHS == 2){            // Verificamos sea AN1 el canal seleccionado
                CCPRB = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR2L = (uint8_t)(CCPRB>>2);    // Guardamos los 8 bits mas significativos en CPR2L
                CCP2CONbits.DC2B0 = CCPRB & 0b11; // Guardamos los 2 bits menos significativos en DC2B
        }
            if(ADCON0bits.CHS == 3){    // Verificamos sea AN3 el canal seleccionado
                pot3 = ADRESH & 0b1111111;        // Mostramos ADRESH 
            }
            if(ADCON0bits.CHS == 4){    // Verificamos sea AN4 el canal seleccionado
                pot4 = ADRESH | 0b1000000;         // Mostramos ADRESH 
            }
        PIR1bits.ADIF = 0;          // Limpiamos bandera de interrupci?n
    } 
    return;
}
/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){        
        // ADC para pots
        if(ADCON0bits.GO == 0){             // No hay proceso de conversion
            if(ADCON0bits.CHS == 1){
                ADCON0bits.CHS = 2;
            }
            else if(ADCON0bits.CHS == 2){
                ADCON0bits.CHS = 3;
            }
            else if(ADCON0bits.CHS == 3){
                ADCON0bits.CHS = 4;
                // cambio en el selector (SS) para generar respuesta del pic
                PORTAbits.RA5 = 1;      // Deshabilitamos el ss del esclavo
                __delay_ms(10);         // Delay para que el PIC pueda detectar el cambio en el pin
                PORTAbits.RA5 = 0;      // habilitamos nuevamente el escalvo

                // Enviamos el dato 
                SSPBUF = pot3;   // Cargamos valor del contador al buffer
                while(!SSPSTATbits.BF){}

                __delay_ms(100);
            }
            else if(ADCON0bits.CHS == 4){
                ADCON0bits.CHS = 0;
                // cambio en el selector (SS) para generar respuesta del pic
                PORTAbits.RA5 = 1;      // Deshabilitamos el ss del esclavo
                __delay_ms(10);         // Delay para que el PIC pueda detectar el cambio en el pin
                PORTAbits.RA5 = 0;      // habilitamos nuevamente el escalvo

                // Enviamos el dato 
                SSPBUF = pot4;   // Cargamos valor del contador al buffer
                while(!SSPSTATbits.BF){}

                __delay_ms(100);
            }
            __delay_us(40);
            ADCON0bits.GO = 1;              // Iniciamos proceso de conversi�n
        }
        
        
    }
    return;
}
/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0b11110;
    ANSELH = 0;
    
    TRISD = 0;
    PORTD = 0;
    
    TRISA = 0b00011111;
    PORTA = 0;
    
    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Reloj interno
    
    // Configuracion de SPI
    // Configs de Maestro
    
        TRISC = 0b00010000;         // -> SDI entrada, SCK y SD0 como salida
        PORTC = 0;
    
        // SSPCON <5:0>
        SSPCONbits.SSPM = 0b0000;   // -> SPI Maestro, Reloj -> Fosc/4 (250kbits/s)
        SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
        SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
        // SSPSTAT<7:6>
        SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
        SSPSTATbits.SMP = 1;        // -> Dato al final del pulso de reloj
        SSPBUF = cont_master;              // Enviamos un dato inicial
    
        // ADC
        ADCON0bits.ADCS = 0b01;     // Fosc/8
        ADCON1bits.VCFG0 = 0;       // VDD
        ADCON1bits.VCFG1 = 0;       // VSS
        ADCON0bits.CHS = 1;    // Seleccionamos los canales
        ADCON0bits.CHS = 2;    // Seleccionamos los canales
        ADCON0bits.CHS = 3;
        ADCON0bits.CHS = 4;
        ADCON1bits.ADFM = 0;        // Justificado a la izquierda
        ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
        __delay_us(40);  
        
        // Configuraci�n PWM
        TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
        PR2 = 30;                  // periodo de 2ms

        // Configuraci�n CCP
        CCP1CON = 0;                // Apagamos CCP1
        CCP1CONbits.P1M = 0;        // Modo single output
        CCP1CONbits.CCP1M = 0b1100; // PWM

        CCPR1L = 16>>2;
        CCP1CONbits.DC1B0 = 16 & 0b11;    // 0.25ms ancho de pulso / 25% ciclo de trabajo

        PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
        T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
        T2CONbits.TMR2ON = 1;       // Encendemos TMR2
        while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
        PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente

        TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM


        // Configuraci�n PWM CCP2
        TRISCbits.TRISC1 = 1;       // Deshabilitamos salida de CCP2

        // Configuraci�n CCP2
        CCP2CON = 0;                // Apagamos CCP2
        CCP2CONbits.CCP2M = 0b1100; // PWM

        CCPR2L = 16>>2;
        CCP2CONbits.DC2B0 = 16 & 0b11;    // 0.25ms ancho de pulso / 25% ciclo de trabajo

        TRISCbits.TRISC1 = 0;       // Habilitamos salida de PWM
        
        // Configuracion interrupciones
        PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
        PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
        INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
        INTCONbits.GIE = 1;         // Habilitamos int. globales
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}