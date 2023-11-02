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

#define FOSC 7372800.0 //Hz Clock breadboard
#define REG_SXT_BIT 65535.0 // MAX 16 bit register
#define TIMER1 1
#define TIMER2 2
void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
void tmr_wait_ms(int timer, int ms);
int set_ledValue(int led);

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
    if (timer == TIMER1){
        while(IFS0bits.T1IF == 0){
        // Wait until the flag is high -> The timer finished to count
        }
    }
    if (timer == TIMER2){ // CONTROLLARE T2
      while(IFS0bits.T2IF == 0){
        // Wait until the flag is high -> The timer finished to count
        }  
    }
    
}

void tmr_wait_ms(int timer, int ms){
    tmr_setup_period(timer, ms);
    if (timer == TIMER1){
        IFS0bits.T1IF = 0; // Reset the flag
        T1CONbits.TON = 1; // Starts the timer
    }
    if (timer == TIMER2){ // CONTROLLARE T2
        IFS0bits.T2IF = 0; // Reset the flag
        T2CONbits.TON = 1; // Starts the timer
    }
    tmr_wait_period(timer);
}

int set_ledValue(int led){
    // change the variable ledValue and switch on or off the led
    if(led == 0){
        LATBbits.LATB0 = 1;
        return 1;
    }
    else{
        LATBbits.LATB0 = 0;
        return 0;
    }
}

int main(void) {
    // Fosc = 7372800 -> Fcy = Fosc / 4 = 1843200 are the number of clocks in one seconds -> too much for 16 bit register (max 65535)
    // With a prescaler 1:8 -> 230400 clocks steps -> too much
    // With a prescaler 1:64 -> 28800 clocks steps -> OK
    // In 1ms -> 28.800 clocks steps
    int ledValue = 0;
    // Presetting
    TRISBbits.TRISB0 = 0; // set the pin B02 as output
    
    // PROBLEM 3
    while(1){
        ledValue = set_ledValue(ledValue);
        tmr_wait_ms(TIMER1,1000); //turn on the led for 1 second
        ledValue = set_ledValue(ledValue);
        tmr_wait_ms(TIMER1,5000); //turn off the led for 5 second
        ledValue = set_ledValue(ledValue);
        tmr_wait_ms(TIMER1,500); //turn on the led for 0.5 second
        ledValue = set_ledValue(ledValue);
        tmr_wait_ms(TIMER1,500); //turn off the led for 0.5 second
    }
    
    // PROBLEM 2
    tmr_setup_period(TIMER1, 500); // Set the PR value and the prescaler
    while(1){
        ledValue = set_ledValue(ledValue);
        IFS0bits.T1IF = 0; // Reset the flag
        T1CONbits.TON = 1; // Starts the timer
        tmr_wait_period(TIMER1);
    }
    return 0;
}