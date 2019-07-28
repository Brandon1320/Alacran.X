/*
 * File:   Alacran.c
 * Author: Brandon Sánchez
 *
 * Created on 24 de Julio de 2019, 12:28 AM
 */

#include <xc.h>
#include <stdlib.h>

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable (All VCAP pin functionality is disabled)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

#define _XTAL_FREQ 32000000
#define SERVO1 LATB0
#define SERVO2 LATB1
#define SERVO3 LATB2
#define SERVO4 LATB3
#define SERVO5 LATB4    
#define NUM_SERVOS 5
#define PULSO_MAX 65536
#define DIECISEIS 16

float pulso_servos[NUM_SERVOS] = {0, 0, 0, 0, 0};
int servo;

void __interrupt() servos() {
    if (PIR1bits.TMR1IF) {

        switch (servo) {
            case 0:
                SERVO1 = 1 - SERVO1;
                if (SERVO1) TMR1 = PULSO_MAX - ((pulso_servos[0] * _XTAL_FREQ) / DIECISEIS);
                else {
                    TMR1 = PULSO_MAX - ((((0.02 / NUM_SERVOS) - pulso_servos[0]) * _XTAL_FREQ) / DIECISEIS);
                    servo++;
                }

                break;
            case 1:
                SERVO2 = 1 - SERVO2;
                if (SERVO2) TMR1 = PULSO_MAX - ((pulso_servos[1] * _XTAL_FREQ) / DIECISEIS);
                else {
                    TMR1 = PULSO_MAX - ((((0.02 / NUM_SERVOS) - pulso_servos[1]) * _XTAL_FREQ) / DIECISEIS);
                    servo++;
                }

                break;
            case 2:
                SERVO3 = 1 - SERVO3;
                if (SERVO3) TMR1 = PULSO_MAX - ((pulso_servos[2] * _XTAL_FREQ) / DIECISEIS);
                else {
                    TMR1 = PULSO_MAX - ((((0.02 / NUM_SERVOS) - pulso_servos[2]) * _XTAL_FREQ) / DIECISEIS);
                    servo++;
                }

                break;
            case 3:
                SERVO4 = 1 - SERVO4;
                if (SERVO4) TMR1 = PULSO_MAX - ((pulso_servos[3] * _XTAL_FREQ) / DIECISEIS);
                else {
                    TMR1 = PULSO_MAX - ((((0.02 / NUM_SERVOS) - pulso_servos[3]) * _XTAL_FREQ) / DIECISEIS);
                    servo++;
                }

                break;
            case 4:
                SERVO5 = 1 - SERVO5;
                if (SERVO5) TMR1 = PULSO_MAX - ((pulso_servos[4] * _XTAL_FREQ) / DIECISEIS);
                else {
                    TMR1 = PULSO_MAX - ((((0.02 / NUM_SERVOS) - pulso_servos[4]) * _XTAL_FREQ) / DIECISEIS);
                    servo = 0;
                }
        }

        PIR1bits.TMR1IF = 0;
    }
}

void main(void) {
    // Oscilador
    OSCCONbits.IRCF = 0b1110;
    OSCCONbits.SCS = 0b00;
    ////

    // Configuracion del puerto
    TRISA = 0;
    PORTA = 1;
    TRISB = 0;
    PORTB = 0;

    // Configuracion interrupcion
    PIE1bits.TMR1IE = 1; // Hanilita la interrupcion por timer
    PIR1bits.TMR1IF = 0; // Limpia la bandera del timer 1
    INTCONbits.GIE = 1; // Interrupcion Global
    INTCONbits.PEIE = 1; // Interrupcion por periferico

    // Configurar timer
    T1CONbits.T1CKPS = 0b10; // Divisor 4
    T1CONbits.TMR1CS = 0b00; // Fosc /4
    T1CONbits.T1OSCEN = 0; // Oscilador LP deshabilitado
    T1CONbits.nT1SYNC = 1; // No sincroniza

    T1CONbits.TMR1ON = 1; // Encender el timer

    while (1) {

        if (PORTAbits.RA1 == 0) {
            PIR1bits.TMR1IF = 0;
            
            servo = 4;
            pulso_servos[4] = 0.0020;
            __delay_ms(100);

            pulso_servos[4] = 0.0010;
            __delay_ms(100);
        } else {
            servo = 0;
            pulso_servos[0] = 0.0010;
            __delay_ms(100);

            servo = 1;
            pulso_servos[1] = 0.0010;
            __delay_ms(100);

            servo = 2;
            pulso_servos[2] = 0.0010;
            __delay_ms(100);

            servo = 3;
            pulso_servos[3] = 0.0010;
            __delay_ms(100);


            servo = 0;
            pulso_servos[0] = 0.0020;
            __delay_ms(100);

            servo = 1;
            pulso_servos[1] = 0.0020;
            __delay_ms(100);

            servo = 2;
            pulso_servos[2] = 0.0020;
            __delay_ms(100);

            servo = 3;
            pulso_servos[3] = 0.0020;
            __delay_ms(100);
            
            
            __delay_ms(200);
        }
    }

    return;
}
