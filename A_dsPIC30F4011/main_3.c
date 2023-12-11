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

/*
 * File:   main_3.c
 * Author: galvd
 *
 * Created on 10 ottobre 2023, 17.15
 */

#include "xc.h"

#define FOSC 7372800.0 //Hz Clock breadboard
#define REG_SXT_BIT 65535.0 // MAX 16 bit register
#define TIMER1 1
#define TIMER2 2
#define LEDD3 1
#define LEDD4 2
int control_interrupt1 = 0;
void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
int set_ledValue(int led, int ledValue);

void tmr_setup_period(int timer, int ms){ // Set the prescaler and the PR value
// Fosc = 737280 Hz -> Fcy = Fosc / 4 = 184320 number of clocks in one second so in 0.1 secon there would be 184320 clocks steps
// this is too high to be put in a 16 bit register (max 65535)
// If we set a prescaler of 1:8 we have 184320/8 = 23040 clock steps
    double Fcy = FOSC / 4.0;
    double clock_steps = Fcy * (ms/1000.0);
    int count = 0;
    if (clock_steps > REG_SXT_BIT){
        double prescaler_value[3] = {8.0,8.0,4.0};
        while(clock_steps > REG_SXT_BIT){
            clock_steps = clock_steps/prescaler_value[count];
            count +=1;           
        }
    }
    if (timer == TIMER1){
        TMR1 = 0.0; // Reset the timer
        PR1 = clock_steps; // Set the desired count value
        T1CONbits.TCKPS = count; // set PRESCALER
    }
    if (timer == TIMER2){
        TMR2 = 0.0;
        PR2 = clock_steps; 
        T2CONbits.TCKPS = count; // set PRESCALER
    }
}

void tmr_wait_period (int timer) {
    if(timer == TIMER1){
        while(IFS0bits.T1IF == 0){
            // Wait until the flag is high -> The timer finished to count
        }
    }
    
    if(timer == TIMER2){
       while(IFS0bits.T2IF == 0){
            // Wait until the flag is high -> The timer finished to count
        } 
    }
        
}

int set_ledValue(int led, int ledValue){
    // change the variable ledValue and switch on or off the led
    if (led == LEDD3){ //LED D3
        if(ledValue == 0){
            LATBbits.LATB0 = 1;
            return 1;
        }
        else{
            LATBbits.LATB0 = 0;
            return 0;
        }
    }
    
    if (led == LEDD4){ //LED D4
        if(ledValue == 0){
            LATBbits.LATB1 = 1; 
            return 1;
        }
        else{
            LATBbits.LATB1 = 0; 
            return 0;
        }
    }
    
} 

void __attribute__((__interrupt__, __auto_psv__)) _INT0Interrupt
    (){
    IFS0bits.INT0IF = 0; // reset interrupt flag
    
    if (control_interrupt1 == 0){
        control_interrupt1 = 1;
    }
    else if (control_interrupt1 == 1){
        control_interrupt1 = 0;
    }
    
}

int main(void) {
    IEC0bits.INT0IE = 1; // enable INT0 interrupt
    // Presetting
    TRISBbits.TRISB0 = 0; // set the pin B02 as output
    TRISBbits.TRISB1 = 0; // set the pin B01 as output
    int ledValue1 = 0;
    int ledValue2 = 0;
    // rest of the code
    //PROBLEM 1
    tmr_setup_period(TIMER1, 500); // Set the PR value and the prescaler
    tmr_setup_period(TIMER2, 250); // set the timer 2 value
    while(1){
        if (control_interrupt1 == 0){
            ledValue2 = set_ledValue(LEDD4, 1); // turn off led 2
            ledValue1 = set_ledValue(LEDD3, ledValue1);
            IFS0bits.T1IF = 0; // Reset the flag
            T1CONbits.TON = 1; // Starts the timer
            tmr_wait_period(TIMER1); 
        }
        else if (control_interrupt1 == 1){
            ledValue1 = set_ledValue(LEDD3, 1); // turn of led 1
            ledValue2 = set_ledValue(LEDD4, ledValue2);
            IFS0bits.T2IF = 0; // Reset the flag
            T2CONbits.TON = 1; // Starts the timer
            tmr_wait_period(TIMER2);
        }
        
    }
    
    return 0;
}
