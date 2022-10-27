//******************************************************************************
//   UNIVERSIDAD DEL VALLE DE GUATEMALA
//   IE2023 PROGRAAMACIÓN DE MICROCONTROLADORES 
//   AUTOR: JORGE SILVA
//   COMPILADOR: XC8 (v1.41), MPLAB X IDE (v6.00)
//   PROYECTO: LABORATORIO 5, PWM
//   HARDWARE: PIC16F887
//   CREADO: 18/10/2022
//   ÚLTIMA MODIFCACIÓN: 24/10/2022
//******************************************************************************
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT
#pragma config WDTE = OFF       
#pragma config PWRTE = ON      
#pragma config MCLRE = OFF      
#pragma config CP = OFF        
#pragma config CPD = OFF        
#pragma config BOREN = OFF      
#pragma config IESO = OFF       
#pragma config FCMEN = OFF     
#pragma config LVP = OFF       

// CONFIG2
#pragma config BOR4V = BOR40V   
#pragma config WRT = OFF
//******************************************************************************
#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 500000
#define tmr0_value 240      //TMR0 a 2ms

void setup(void);
void paso(int voltaje);
void setupADC(void);
void setupPWM(void);

uint8_t cont;
int volt; 
int SERVO1;
int SERVO2; 
int LED;


//******************************************************************************
// INTERRUPCIONES
//******************************************************************************
void __interrupt() isr (void){
    if (INTCONbits.T0IF){ //TMR0 PARA EL LED
        cont++;
        if (cont <= LED){
            PORTCbits.RC7 = 1; 
        }
        else{
            PORTCbits.RC7 = 0; 
        }
        TMR0 = tmr0_value; 
        INTCONbits.T0IF = 0;
    }
}
//******************************************************************************
// Código Principal
//******************************************************************************
void main(void) {
    
    setup();
    setupADC();
    setupPWM();
    cont = 0; 
    
    while(1){
        //SERVO1
        ADCON0bits.CHS = 0b0001;
        __delay_us(100);
        ADCON0bits.GO = 1;
        while (ADCON0bits.GO == 1){
            ;
        }
        SERVO1 = ADRESH;
        paso(SERVO1);
        CCPR1L = volt; 
        __delay_us(100);
        
        //SERVO2
        ADCON0bits.CHS = 0b0010;  
        __delay_us(100);
        ADCON0bits.GO = 1; 
        while (ADCON0bits.GO == 1){
            ;
        }
        SERVO2 = ADRESH; 
        paso(SERVO2);
        CCPR2L = volt; 
        __delay_us(100);
        
        //LED
        ADCON0bits.CHS = 0b0011;  
        __delay_us(100);
        ADCON0bits.GO = 1; 
        while (ADCON0bits.GO == 1){
            ;
        }
        LED = ADRESH;  
        __delay_us(100);
    }
}
//******************************************************************************
// Función para la conversión de voltaje
//******************************************************************************
void paso(int voltaje){
    volt = (unsigned short)(7+((float)(9)/(255))*(voltaje-0));
}
//******************************************************************************
// Función para configurar GPIOs
//******************************************************************************
void setup(void){
    //Puertos
    ANSELH = 0; //Digitales
    TRISB = 0; //Salida
    PORTB = 0;
    TRISC = 0;
    PORTC = 0;
    
    //OSCILADOR
    OSCCONbits.IRCF = 0b011;       // 500 KHz
    OSCCONbits.SCS = 1;
    
    //INTERRUPCIONES
    INTCONbits.GIE = 1;
    
    PIE1bits.ADIE = 1;              //ADC
    PIR1bits.ADIF = 0;              
    
    INTCONbits.TMR0IE = 1;          //TMR0
    INTCONbits.T0IF = 0;
    OPTION_REGbits.T0CS = 0;     
    OPTION_REGbits.PSA = 0;     
    OPTION_REGbits.PS = 0b011;      //Prescaler 1:16
    TMR0 = tmr0_value;              
    
}
//******************************************************************************
// Función para configurar ADC
//******************************************************************************
void setupADC(void){
    
    // Paso 1 Seleccionar puertoS de entrada
    TRISAbits.TRISA0 = 1;
    ANSELbits.ANS0 = 1;
    
    TRISAbits.TRISA1 = 1;
    ANSELbits.ANS1 = 1; 
    
    TRISAbits.TRISA2 = 1;
    ANSELbits.ANS2 = 1; 
  
    
    // Paso 2 Configurar módulo ADC
    
    ADCON0bits.ADCS1 = 0;
    ADCON0bits.ADCS0 = 1;       // Fosc/ 8
    
    ADCON1bits.VCFG1 = 0;       // Ref VSS
    ADCON1bits.VCFG0 = 0;       // Ref VDD
    
    ADCON1bits.ADFM = 0;        // Justificado hacia izquierda
    
    //ADCON0bits.CHS3 = 0;
    //ADCON0bits.CHS2 = 0;
    //ADCON0bits.CHS1 = 0;
    //ADCON0bits.CHS0 = 0;        // Canal AN0
    //ADCON0bits.CHS = 0b0001;    // Canal AN1
    ADCON0bits.CHS = 0b0010;      // Canal AN2
    //ADCON0bits.CHS = 0b0011;
    
    ADCON0bits.ADON = 1;        // Habilitamos el ADC
    __delay_us(100);
    
}
//******************************************************************************
// Función para configurar PWM
//******************************************************************************
void setupPWM(void){
    // Paso 1. Definir pines de donde saldrá la señal del PWM
    TRISCbits.TRISC1 = 1;
    TRISCbits.TRISC2 = 1;
    
    // Paso 2. Definimos periodo
    PR2 = 255;      // 20mS
    
    // Paso 3. Configuramos los bits de salida/entrada del PWM
    
    CCP1CONbits.P1M = 0b00;         //Single output 
    
    CCP1CONbits.CCP1M = 0b1100;     //P1A como PWM 
    
    CCP2CONbits.CCP2M = 0b1111;     //P2A como PWM
            
   // Paso 4. Ancho de pulso
    CCP1CONbits.DC1B = 0b11;        // CCPxCON<5:4>
    CCPR1L = 11;        // CCPR1L 
                        // CALCULO PARA 1.5mS de ancho de pulso
    
    //CCP1CONbits.DC2B = 0b11;        // CCPxCON<5:4>
    CCP2CONbits.DC2B1 = 0b1;       
    CCP2CONbits.DC2B0 = 0b1;
    CCPR1L = 11;        // CCPR1L 
                        // CALCULO PARA 1.5mS de ancho de pulso
    
    // Paso 5. Configurar TMR2
    TMR2IF = 0;
    T2CONbits.T2CKPS = 0b11;      // Prescaler de 1:16
    TMR2ON = 1;         // Encender timer 2
    
    // Paso 6. Que hacer con el TMR2
    while(!TMR2IF); //Mientras esté apagado
    TRISCbits.TRISC1 = 0;   // Habilitamos la salida del PWM
    TRISCbits.TRISC2 = 0;   // Habilitamos la salida del PWM
    
}