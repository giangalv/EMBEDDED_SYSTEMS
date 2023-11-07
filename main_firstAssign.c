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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define FOSC 7372800.0 //Hz Clock breadboard
#define REG_SXT_BIT 65535.0 // MAX 16 bit register
#define TIMER1 1
#define TIMER2 2
#define TIMER3 3
#define SIZE_OF_BUFFER 64

void UART2_Init();
void SPI1_Init();
void algorithm();
void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
void printFunctionFirstRow(char receivedChar);

int number_readings = 0; // global variable for counting all the printed value
int number_first_raw = 0; // global variable for counting the printed value on the first raw
int position_first_raw[16] = {0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x8B,0x8C,0x8D,0x8E,0x8F};
/*
int bufferLength = 0; // writeIndex - readIndex
int readIndex = 0;
int writeIndex = 0;
char cb[SIZE_OF_BUFFER]; // size buffer
*/

// TIMER FUNCTIONS
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
        }
    }   
    else if(timer == TIMER2){
        while(IFS0bits.T2IF == 0){
            // Wait until the flag is high -> The timer finished to count
        }
    }
    else if(timer == TIMER3){
        while(IFS0bits.T3IF == 0){
            // Wait until the flag is high -> The timer finished to count
        }
    }
}
void tmr_wait_ms(int timer, int ms){
    tmr_setup_period(timer, ms);
    if (timer == TIMER3){
        IFS0bits.T3IF = 0; // Reset the flag
        T3CONbits.TON = 1; // Starts the timer
    }
    tmr_wait_period(timer);
}

void algorithm() {
    IFS0bits.T2IF = 0; // Reset the flag TIMER 2
    T2CONbits.TON = 1; // Starts the timer TIMER 2
    tmr_wait_period(TIMER2);
}

// INIT FUNCTIONS
void SPI1_Init(){
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8-bit mode
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE = 3; // 5:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI
}
void UART2_Init() {
    U2MODE = 0;             // Clear the mode register
    U2STA = 0;              // Clear the status and control register
    U2BRG = 11;             // Baud Rate Generator value for a specific baud rate 
    // (e.g., 9600 bps with a 7.3728 MHz clock, Fcy = 1.8432 MHz) || (7372800 / 4) / (16 * 9600) - 1
    U2MODEbits.UARTEN = 1;  // Enable UART
    U2STAbits.UTXEN = 1;    // Enable UART transmitter
}
void initFunctionSecondRaw(){
    // Send a control command to set the cursor position
    setCursorPositionSecondROw();
    char initSecondRaw[11] = {'C','h', 'a', 'r' ,' ', 'R', 'e', 'c', 'v', ':', ' '}; //scrivo dal c10 o c11
    int i = 0;
    while(i<11){
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF = initSecondRaw[i];
        i++;
    }
}

// SETTING CURSOR FUNCTIONS
void setCursorPositionFirstROw(int position) {
    // Send a control command to set the cursor position
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = position;
    IFS0bits.T1IF = 0; // Reset the flag
    T1CONbits.TON = 1; // Starts the timer    
    tmr_wait_ms(TIMER3,1); // wait 1ms after moving the cursor
}
void setCursorPositionSecondROw() {
    // Send a control command to set the cursor position
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0xC0;
    IFS0bits.T1IF = 0; // Reset the flag
    T1CONbits.TON = 1; // Starts the timer
    tmr_wait_ms(TIMER3,1); // wait 1ms after moving the cursor
}

// PRINTING/CLEANING FUNCTIONS
void cleaningFirstRow(){
    char cleaner = ' ';
    setCursorPositionFirstROw(0x80);
    int i = 0;
    while(i<16){
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF = cleaner;
        i++;
    }
}

void printFunctionFirstRow(char receivedChar){
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = receivedChar;
    number_first_raw++;
    number_readings++;
    
    convertNumberToString(number_readings);
    setCursorPositionFirstROw(position_first_raw[number_first_raw]);
    
    if(receivedChar=='\r'||receivedChar=='\n'){
        //function to clean the first raw;
        cleaningFirstRow();
        number_first_raw = 0;
        setCursorPositionFirstROw(0x80);   
    }  
    
    if(number_first_raw==16){
        //clean first raw
        cleaningFirstRow();
        number_first_raw = 0;
        setCursorPositionFirstROw(0x80);
        // setCursorPosition(0xC0); // move at the beginning of the second row
    } 
}

void print_function(char printed_value[]){
    // Send a control command to set the cursor position
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0xCA;
    IFS0bits.T1IF = 0; // Reset the flag
    T1CONbits.TON = 1; // Starts the timer
    tmr_wait_ms(TIMER3,1); // wait 1ms after moving the cursor
    
    int i = 0;
     while(1){       
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF = printed_value[i];
               
        int end = strlen(printed_value)-1;
        if (i==end){
            return;
        }
        i++; 
    }
}

void convertNumberToString(int count){ 
    // clean
    char clr[19] = {' ', ' ', ' ',' ', ' ', ' '};
    print_function(clr); 
    // read
    char str[10];
    sprintf(str, "%d", count);
    print_function(str);
}



/*
void __attribute__((__interrupt__, __auto_psv__)) _INT0Interrupt
    (){
    IFS0bits.INT0IF = 0; // reset interrupt flag
    
    // Manage the losing bytes 
    
}
*/

// INTERRUPTS
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(void) {
    // Pulisci il flag dell'interrupt.
    IFS1bits.U2RXIF = 0;
    
    if(U2STAbits.URXDA == 1){
        // Leggi il dato dal registro di ricezione UART2.
        char receivedData = U2RXREG;
        push(receivedData);
    }
}

// CIRCULAR BUFFER
struct circular_buffer {
    char buffer[SIZE_OF_BUFFER]; // size buffer
    int bufferLength;        // writeIndex - readIndex
    int readIndex;
    int writeIndex;
};
struct circular_buffer cb;
void initBuffer(){
    cb.bufferLength = 0; // writeIndex - readIndex
    cb.readIndex = 0;
    cb.writeIndex = 0;
}
void push(char receivedChar){
    if (cb.bufferLength == SIZE_OF_BUFFER)
    {
        //printf(?Buffer is full!?);
    }
    else{
        cb.buffer[cb.writeIndex] = receivedChar;
        cb.bufferLength++;
        cb.writeIndex++;
        if (cb.writeIndex == SIZE_OF_BUFFER) 
        {
            cb.writeIndex = 0;
        }
    }
}
char pull(){
    char empty = ' ';
    if (cb.bufferLength == 0) 
    {
        //printf(?Buffer is empty!?);
        return empty;
    }
    else{
        char receivedChar = cb.buffer[cb.readIndex];
        cb.bufferLength--;
        cb.readIndex++;
        if (cb.readIndex == SIZE_OF_BUFFER) 
        {
            cb.readIndex = 0;
        }
        return receivedChar;
    }
}

int main(void) {
    IEC1bits.U2RXIE = 1; // Abilita l'interrupt per la ricezione UART2
    U2STAbits.URXISEL = 3; // Set URXISEL to 11
    SPI1_Init(); // initializd SPI1
    UART2_Init(); // initialize UART2
       
    tmr_setup_period(TIMER1, 10); // Set the PR value and the prescaler (TIMER1 for allowing LCD to display values)
    tmr_setup_period(TIMER2, 7); // Set the PR value and the prescaler (TIMER2 responsible of algorithm)
    tmr_setup_period(TIMER3, 3000);
    
    initBuffer();
    
    char receivedChar;
    // STARTING THE LCD
    IFS0bits.T3IF = 0; // Reset the flag
    T3CONbits.TON = 1; // Starts the timer
    tmr_wait_period(TIMER3); 
    initFunctionSecondRaw();
    convertNumberToString(number_readings); // DA FARE
    setCursorPositionFirstROw(0x80);
    while(1){
        algorithm();
        
        if(cb.bufferLength > 0){
            receivedChar = pull();
            printFunctionFirstRow(receivedChar);
        }

        IFS0bits.T1IF = 0; // Reset the flag
        T1CONbits.TON = 1; // Starts the timer
        tmr_wait_period(TIMER1); 
    }
}
