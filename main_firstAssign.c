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

#include "xc.h"

#define FOSC 7372800.0 //Hz Clock breadboard
#define REG_SXT_BIT 65535.0 // MAX 16 bit register
#define TIMER1 1
#define TIMER2 2
void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);

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
    else if(timer == TIMER2){
        while(IFS0bits.T2IF == 0){
            // Wait until the flag is high -> The timer finished to count
        }
    }
}

int print_function(char printed_value[]){
    int number_count = number_readings; // local variable that takes the number of character received until now
    int i = 0;
     while(1){
        IFS0bits.T1IF = 0; // Reset the flag
        T1CONbits.TON = 1; // Starts the timer
        tmr_wait_period(TIMER1);
        
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF = printed_value[i];
        if(printed_value[i]=='\r'||printed_value[i]=='\n'){
            //function to clean the first raw;
            //function to write the number on the second raw
            second_raw(number_count);
        }
        
        number_count++;
        i++;
        int end = sizeof(printed_value[]);
        if (i==end){
            return;
        }
        IFS0bits.T1IF = 0; // Reset the flag
        T1CONbits.TON = 1; // Starts the timer
        tmr_wait_period(TIMER1);
    }
}

void second_raw(int number_count){
    SPI1BUF = '0xC0';
    int i = 0;
    char str[1000];
     while(1){
        IFS0bits.T1IF = 0; // Reset the flag
        T1CONbits.TON = 1; // Starts the timer
        tmr_wait_period(TIMER1);
        
        char printed_value[11] = {'C','h', 'a', 'r' ,' ', 'R', 'e', 'c', 'v', ':'};
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF = printed_value[i];
        
        i++;
        int end = sizeof(printed_value[]);
        if (i==end){
            sprintf(str[j],"%d",number_count);
            print_function(str[]);
            return;
        }
        IFS0bits.T1IF = 0; // Reset the flag
        T1CONbits.TON = 1; // Starts the timer
        tmr_wait_period(TIMER1);
    }
}

void algorithm() {
    IFS0bits.T2IF = 0; // Reset the flag TIMER 2
    T2CONbits.TON = 1; // Starts the timer TIMER 2
    tmr_wait_period(TIMER2);
}
/*
int main(void) {
    SPI1CONbits.MSTEN = 1;
    SPI1CONbits.MODE16 = 0;
    SPI1CONbits.PPRE = 3;
    SPI1CONbits.SPRE = 3;
    SPI1STATbits.SPIEN = 1;
    
    tmr_setup_period(TIMER1, 500); // Set the PR value and the prescaler
   
    // Set a timer to expire ewvery second; write the seconds elapsed on
    // the LCD (use sprintf(buffer, ?%d?, value) to convert an integer to
    // a string to be displayed
    char str[1000];
    int count = 0;
    while(1){
        int digitsnumber = log10(count)+1;
        // create the vector to print the number
        for(int j=0; j<(digitsnumber+1); j++){
            int num = count;  
            num = '0' + (count % 10); // take out the last digit and convert it into a char
            sprintf(str[j],"%d",num);
            num /= 10; // remove the last digit    
        }
        print_function(str[]);
        
        // create the vector to clear the LCD
        for(int j=0; j<(digitsnumber+1); j++){
            str[j] = '';   
        }
        print_function(str[]);   
        
        //increase the number 
        count ++;
    }
    
    // Write ?HELLO WORLD? on the LCD display 
    char printed_value[11] = {'H','e', 'l', 'l' ,'o', ' ', 'w', 'o', 'r', 'l', 'd'};
    int i = 0;
    while(1){
        IFS0bits.T1IF = 0; // Reset the flag
        T1CONbits.TON = 1; // Starts the timer
        tmr_wait_period(TIMER1);
        
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF = printed_value[i];
        
        i++;
        if (i==11){
             i=0;
        }
        IFS0bits.T1IF = 0; // Reset the flag
        T1CONbits.TON = 1; // Starts the timer
        tmr_wait_period(TIMER1);
    }
    return 0;
}
*/
void UART2_Init() {
    U2MODE = 0;             // Clear the mode register
    U2STA = 0;              // Clear the status and control register

    U2BRG = 25;             // Baud Rate Generator value for a specific baud rate (e.g., 9600 bps with a 40 MHz clock)
    U2MODEbits.UARTEN = 1;  // Enable UART
    U2STAbits.UTXEN = 1;    // Enable UART transmitter
}

char UART2_Read() {
    while (!U2STAbits.URXDA);  // Wait for data to be received

    return U2RXREG;  // Return the received character
}

int number_readings = 0; // global variable

int main(void) {
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8-bit mode
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE = 3; // 5:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI
    tmr_setup_period(TIMER2, 7); // Set the PR value and the prescaler (TIMER1 responsible of algorithm)
    tmr_setup_period(TIMER1, 500); // Set the PR value and the prescaler (TIMER2 for allowing LCD to display values)
    
    UART2_Init(); // initialize UART2
    char receivedChar[100];

    
    while(1){
        algorithm();
        // read character from UART2
        receivedChar = UART2_Read();
        // display on LCD
        print_function(receivedChar); // to add Control for first raw fullness 
        number_readings += sizeof(receivedChar);
        
    }
    return 0;
}
