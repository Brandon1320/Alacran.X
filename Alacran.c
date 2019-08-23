#include <xc.h>
#define _XTAL_FREQ 32000000
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

#define servo1 LATB0
#define servo2 LATB1

#define num_servos 2

int servo;
float pulso_servos[num_servos]={0,0};

void interrupt servos(){
       
    if (PIR1bits.TMR1IF){
    
     if (servo==0){
       servo1=1-servo1;
       if (servo1) TMR1=65536-((pulso_servos[0] *_XTAL_FREQ)/16);  
       else {TMR1=65536-((((0.02/num_servos)-pulso_servos[0])*_XTAL_FREQ)/16); servo++;}   
     }   
     
     if (servo==1){
      servo2=1-servo2;
      if (servo2) TMR1=65536-((pulso_servos[1] *_XTAL_FREQ)/16);
      else {TMR1=65536-((((0.02/num_servos)-pulso_servos[1])*_XTAL_FREQ)/16); servo = 0;}        
     }
     
     }
     
     PIR1bits.TMR1IF=0;
    
    
}
void main(void) {
   
    //Oscilador
    OSCCONbits.IRCF=0b1110;
    OSCCONbits.SCS=0b00;
    ////
    
    //Configuracion del puerto 
    TRISB=0;
    PORTB=0;
    
    //Configuracion interrupcion
    
    PIE1bits.TMR1IE=1; //Hanilita la interrupcion por timer
    PIR1bits.TMR1IF=0; //Limpia la bandera del timer 1
    INTCONbits.GIE=1; // Interrupcion Global
    INTCONbits.PEIE=1; // Interrupcion por periferico
    //Configurar timer
    
    T1CONbits.T1CKPS=0b10; // Divisor 4
    T1CONbits.TMR1CS=0b00;// Fosc /4
    T1CONbits.T1OSCEN=0; // Oscilador LP deshabilitao
    T1CONbits.nT1SYNC=1; // No sinconia
    
    
    T1CONbits.TMR1ON=1; // Encender el timer
    
    while(1){
        servo=0;
        pulso_servos[0]=0.0010;
        __delay_ms(100);
        
        servo=1;
        pulso_servos[1]=0.0010;
        __delay_ms(100);
       
       servo=0;
        pulso_servos[0]=0.0020;
        __delay_ms(100);
        
        servo=1;
        pulso_servos[1]=0.0020;
        __delay_ms(100);
        
    }
    return;
}
