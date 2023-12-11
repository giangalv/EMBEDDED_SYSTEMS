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
#define TIMER3 3
void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
void reset_total(int value);
int on_off_led(int ledValue);

int ledValue;
int pinValue;
int count;
int cmode;

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
    if (timer == TIMER3){
        TMR3 = 0.0;
        PR3 = clock_steps; 
        T3CONbits.TCKPS = count; // set PRESCALER
    }
}

void tmr_wait_period (int timer) {
    if(timer == TIMER1){
        while(IFS0bits.T1IF == 0){
            // Wait until the flag is high -> The timer finished to count
            pinValue = PORTDbits.RD0; // read from pin (initialized 1)
            if (pinValue == 0){
                T3CONbits.TON = 1; // Starts the timer TIMER 3
                while(pinValue == 0){
                    pinValue = PORTDbits.RD0; // read from pin (initialized 1)
                    if(IFS0bits.T3IF == 1){
                        reset_total(1);
                    }
                }
                reset_total(0);
            }
        }  
    }
    else if(timer == TIMER2){
        while(IFS0bits.T2IF == 0){ // DA CAMBIARE
            // Wait until the flag is high -> The timer finished to count
            pinValue = PORTDbits.RD0; // read from pin (initialized 1)
            if (pinValue == 0){
                T3CONbits.TON = 1; // Starts the timer TIMER 3
                while(pinValue == 0){
                    pinValue = PORTDbits.RD0; // read from pin (initialized 1)
                    if(IFS0bits.T3IF == 1){
                        reset_total(1);
                    }
                }                
                reset_total(0);
            }
        } 
        count = 0;
        TMR2 = 0.0;
        IFS0bits.T2IF = 0;        
    }
}

void reset_total(int value){
    if(value == 0){
        count = 0; // update the count for the loops in the main, and the current mode
        if(cmode == 3){
            cmode = 1;
        }
        else if (cmode == 0){
            cmode = 0;
        }
        else{
            cmode += 1;
        }
        LATBbits.LATB0 = 0;
        ledValue = 0; 
        TMR1 = 0.0; // Reset the timers
        TMR2 = 0.0;
        TMR3 = 0.0;
        IFS0bits.T1IF = 0; // Reset the flags
        IFS0bits.T2IF = 0;
        IFS0bits.T3IF = 0;
    }
    else if(value == 1){
        LATBbits.LATB0 = 0;
        ledValue = 0;
        cmode = 0;
    }
}

int on_off_led(int ledValue){
    if(ledValue == 0){
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
    
    // Presetting
    tmr_setup_period(TIMER1, 100); // Set the PR value and the prescaler
    tmr_setup_period(TIMER2, 1000); // Set the PR value and the prescaler
    tmr_setup_period(TIMER3, 3000); // Set the PR value and the prescaler
    IFS0bits.T3IF = 0; // Reset the flag TIMER 3
    TRISBbits.TRISB0 = 0; // set the pin B02 as output
    TRISDbits.TRISD0 = 1; // set the pin D02 as input
    
    int mode[3] = {2,4,6};
    cmode = 0; // current mode
    count = 0;
    ledValue = 0;
    pinValue = 1;
    
    while(1){
        if(cmode==1){
            ledValue = on_off_led(ledValue);
            IFS0bits.T1IF = 0; // Reset the flag TIMER 1
            T1CONbits.TON = 1; // Starts the timer TIMER 1
            IFS0bits.T3IF = 0; // Reset the flag TIMER 3
            TMR3 = 0.0;
            if(count == 0){
                IFS0bits.T2IF = 0; // Reset the flag TIMER2
                T2CONbits.TON = 1; // Starts the timer TIMER2
            }
            count += 1;
            if (count == mode[cmode-1]){
                tmr_wait_period(TIMER2);
            }
            else{
                tmr_wait_period(TIMER1);
            }
        }
        else if(cmode==2){
            ledValue = on_off_led(ledValue);
            IFS0bits.T1IF = 0; // Reset the flag TIMER 1
            T1CONbits.TON = 1; // Starts the timer TIMER 1
            IFS0bits.T3IF = 0; // Reset the flag TIMER 3
            TMR3 = 0.0;
            if(count == 0){
                IFS0bits.T2IF = 0; // Reset the flag2
                T2CONbits.TON = 1; // Starts the timer2
            }
            count += 1;
            if (count == mode[cmode-1]){
                tmr_wait_period(TIMER2);
            }
            else{
                tmr_wait_period(TIMER1);
            }
        }
        else if(cmode==3){
            ledValue = on_off_led(ledValue);
            IFS0bits.T1IF = 0; // Reset the flag TIMER 1
            T1CONbits.TON = 1; // Starts the timer TIMER 1
            IFS0bits.T3IF = 0; // Reset the flag TIMER 3
            TMR3 = 0.0;
            if(count == 0){
                IFS0bits.T2IF = 0; // Reset the flag2
                T2CONbits.TON = 1; // Starts the timer2
            }
            count += 1;
            if (count == mode[cmode-1]){
                tmr_wait_period(TIMER2);
            }
            else{
                tmr_wait_period(TIMER1);
            }
        }
        else if (cmode==0){
            reset_total(0);
            pinValue = PORTDbits.RD0; // read from pin
            if (pinValue == 0){
                cmode = 3; //start the if blink   
            }           
        }
    }
    return 0;
}