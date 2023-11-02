// DSPIC30F4011 Configuration Bit Settings
// 'C' source line config statements
// FOSC
#pragma config FPR = XT            // Primary Oscillator Mode (XT)
#pragma config FOS = PRI           // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF// Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16   // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512  // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF       // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64  // POR Timer Value (64ms)
#pragma config BODENV = BORV20  // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON  // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI// Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI// High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN// PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN  // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF      // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF  // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD       // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
/*
 * File:   main_1a.c
 * Author: galvd
 *
 * Created on 26 settembre 2023, 12.51
 */

#include "xc.h"

int probA(void){
    TRISBbits.TRISB0 = 0; // set the pin B02 as output
    
    while(1){
        LATBbits.LATB0 = 1; // set the pin high
    }
}

int probB(void){
    int pinValue; // variable to read a value
    TRISDbits.TRISD0 = 1; // set the pin D02 as input
    TRISBbits.TRISB0 = 0; // set the pin B02 as output
    
    while(1) {
        pinValue = PORTDbits.RD0; // read from pin NORMALMENTE CHIUSO!
        if (pinValue == 0){
            LATBbits.LATB0 = 1; // set the pin high
        }
        else {
            LATBbits.LATB0 = 0; // set the pin low
        }    
    }
}

int probC(void){
    int pinValue; // variable to read a value
    TRISDbits.TRISD0 = 1; // set the pin D02 as input
    TRISBbits.TRISB0 = 0; // set the pin B02 as output
    int key = 1; // variable to know the pins' condition 
    LATBbits.LATB0 = 0; // set the led low
    
    while(1){
        pinValue = PORTDbits.RD0; // read from pin NORMALMENTE CHIUSO = 1
        if (pinValue == 0){
            LATBbits.LATB0 = key; // change the leds' status 
            
            while(pinValue != 1){ // wait the inputs' status change
                pinValue = PORTDbits.RD0; 
            }
            if (key == 1){
                key = 0;
            }
            else {
                key = 1;
            }
        }   
    }
}

int main(void) {
    probA();
    //probB();
    //probC();
    return 0;
}




