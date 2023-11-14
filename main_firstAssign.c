// DSPIC30F4011 Configuration Bit Settings
// 'C' source line config statements
// FOSC
#pragma config FPR = XT                   // Primary Oscillator Mode (XT)
#pragma config FOS = PRI                  // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF     // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16          // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512         // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF              // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64            // POR Timer Value (64ms)
#pragma config BODENV = BORV20            // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON            // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI        // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI        // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN         // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN            // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF            // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF        // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD              // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

#include "xc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <p30F4011.h>

#define FOSC 7372800.0                   // Hz Clock breadboard
#define REG_SXT_BIT 65535.0              // MAX 16 bit register
#define TIMER1 1
#define TIMER2 2
#define TIMER3 3
#define TIMER4 4
#define SIZE_OF_BUFFER 16

void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
void tmr_wait_ms(int timer, int ms);
void UART2_Init();
void SPI1_Init();
void algorithm();
void initFunctionSecondRow();
void setCursorPositionFirstRow(int position);
void setCursorPositionSecondRow();
void cleaningFirstRow();
void printFunctionFirstRow(char receivedChar);
void printFunctionSecondRow(char printed_value[]);
void push(char receivedChar);
void push_main(char receivedChar);
void pull();
void checkFlagsInterrupt();

void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt();
void __attribute__((__interrupt__, __auto_psv__)) _INT0Interrupt();
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt();
void __attribute__ ((__interrupt__, __auto_psv__)) _T1Interrupt();

int number_readings = 0;               // global variable for counting all the printed value
int number_first_row = 0;              // global variable for counting the printed value on the first raw
int position_first_row[16] = {0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x8B,0x8C,0x8D,0x8E,0x8F};
int flags_interrupts = 0;              // interrupt flag S5 and S6

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
        TMR1 = 0.0;              // Reset the timer
        PR1 = clock_steps;       // Set the desired count value
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
    if (timer == TIMER4){
        TMR4 = 0.0;
        PR4 = clock_steps; 
        T4CONbits.TCKPS = count; // set PRESCALER
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
    else if(timer == TIMER4){
        while(IFS1bits.T4IF == 0){
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
    // Function to initialize the SPI
    SPI1CONbits.MSTEN = 1;   // master mode
    SPI1CONbits.MODE16 = 0;  // 8-bit mode
    SPI1CONbits.PPRE = 3;    // 1:1 primary prescaler
    SPI1CONbits.SPRE = 3;    // 5:1 secondary prescaler
    SPI1STATbits.SPIEN = 1;  // enable SPI
}
void UART2_Init() {
    // Function to initialize the UART2
    U2MODE = 0;              // Clear the mode register
    U2STA = 0;               // Clear the status and control register
    U2BRG = 11;              // Baud Rate Generator value for a specific baud rate 
    // (e.g., 9600 bps with a 7.3728 MHz clock, Fcy = 1.8432 MHz) || (7372800 / 4) / (16 * 9600) - 1
    U2MODEbits.UARTEN = 1;   // Enable UART
    U2STAbits.UTXEN = 1;     // Enable UART transmitter
}
void initFunctionSecondRow(){
    // Function to initialize the second row: we print the 'Char Recv:' string and hold on it,
    // so we'll only print the updated current number of readings from UART2
    setCursorPositionSecondRow();
    char initSecondRaw[11] = {'C','h', 'a', 'r' ,' ', 'R', 'e', 'c', 'v', ':', ' '}; //scrivo dal c10 o c11
    int i = 0;
    while(i<11){
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF = initSecondRaw[i];
        i++;
    }
}

// SETTING CURSOR FUNCTIONS
void setCursorPositionFirstRow(int position) {
    // Function for setting the position of the cursor when we print on the first row
    IFS0bits.T3IF = 0;               //  WAITING the CURSORs' moving and reset the flag
    T3CONbits.TON = 1;               // Starts the timer
    tmr_wait_period(TIMER3);
    while(SPI1STATbits.SPITBF == 1); // Send a control command to set the cursor position
    SPI1BUF = position;
     
}
void setCursorPositionSecondRow() {
    // Function for setting the position of the cursor when we print on the second row
    while(SPI1STATbits.SPITBF == 1); // Send a control command to set the cursor position
    SPI1BUF = 0xC0;
    IFS0bits.T3IF = 0;               // WAITING the CURSORs' moving and Reset the flag
    T3CONbits.TON = 1;               // Starts the timer
    tmr_wait_period(TIMER3); 
}

// PRINTING/CLEANING FUNCTIONS
void cleaningFirstRow(){
    // Function for cleaning the first row
    char cleaner = ' ';
    setCursorPositionFirstRow(0x80);
    int i = 0;
    while(i<16){
        while(SPI1STATbits.SPITBF == 1);
        SPI1BUF = cleaner;
        i++;
    }
}

void printFunctionFirstRow(char receivedChar){
    // Function for printing the char received from UART2 (passed as a parameter)
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = receivedChar;
    number_first_row++;
    if(receivedChar=='\r'||receivedChar=='\n'){  // If we get a special char, clean the first row
        cleaningFirstRow();
        number_first_row = 0;
        setCursorPositionFirstRow(0x80);   
    }  
    if(number_first_row==16){                    // If the first row is full, clean it
        cleaningFirstRow();
        number_first_row = 0;
        setCursorPositionFirstRow(0x80);
    } 
}

void printFunctionSecondRow(char printed_value[]){
    // Function for printing the current number of received chars from UART2
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0xCA;
    IFS0bits.T3IF = 0;               // WAITING the CURSORs' moving and reset the flag
    T3CONbits.TON = 1;               // Starts the timer
    tmr_wait_period(TIMER3); 
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
    // Function for converting an integer (the count of the number of received chars from UART2) in a char 
    // and then print it on the second row
    char clr[19] = {' ', ' ', ' ',' ', ' ', ' '}; // Clean the positions where we'll print
    printFunctionSecondRow(clr); 
    char str[10];
    sprintf(str, "%d", count);                    // Convert int to char
    printFunctionSecondRow(str);                  // Print
}

// INTERRUPTS
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt
    () {
    // Interrupt function of the UART2: when the buffer is full, execute it.
    // Read the buffer and push the data inside the circular buffer
    IFS1bits.U2RXIF = 0;                // Reset the flag
    if(U2STAbits.URXDA == 1){           // If there are data to be read
        char receivedData = U2RXREG;    // Read
        push(receivedData);             // Push
    }    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void __attribute__((__interrupt__, __auto_psv__)) _INT0Interrupt
    (){
    IFS0bits.INT0IF = 0; // reset interrupt flag
    IFS1bits.T4IF = 0; // Reset the flag
    T4CONbits.TON = 1; // Starts the timer
    tmr_wait_period(TIMER4);
}

void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt
    (){
    IFS1bits.INT1IF = 0; // reset interrupt flag
    flags_interrupts = 2; 
}

void __attribute__ ((__interrupt__, __auto_psv__)) _T1Interrupt
    (){
    IFS0bits.T1IF = 0;            // reset interrupt flag
    if(PORTDbits.RD0 == 1){       // CHECKING if the BOTTON S5 is already pressed
        flags_interrupts = 1;
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
        //BUFFER FULL
        return;
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

void push_main(char receivedChar){
    if (cb.bufferLength == SIZE_OF_BUFFER)
    {
        //BUFFER FULL
        return;
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

void pull(){
	while (cb.bufferLength > 0){
	    if (cb.bufferLength == 0) 
	    {
		//printf(?Buffer is empty!?);
		return;
	    }
	    else{
            char receivedChar = cb.buffer[cb.readIndex];
            cb.bufferLength--;
            cb.readIndex++;
            number_readings++;
            if (cb.readIndex == SIZE_OF_BUFFER) 
            {
                cb.readIndex = 0;
            }
            printFunctionFirstRow(receivedChar);
            if (cb.bufferLength == 0 && U2STAbits.URXDA == 0) 
            {
                convertNumberToString(number_readings);
                setCursorPositionFirstRow(position_first_row[number_first_row]);
            }
	    }
	}
}


// INTERUPT FLAGS
void checkFlagsInterrupt(){
    if(flags_interrupts == 1){
        while (U2STAbits.UTXBF);  // Wait for the transmit buffer to be empty
        U2TXREG = number_readings;  // Send the number of chars received
        flags_interrupts = 0;
    }
    if(flags_interrupts == 2){
        cleaningFirstRow();
        number_readings = 0;
        number_first_row = 0;
        convertNumberToString(number_readings); 
        setCursorPositionFirstRow(0x80);
        flags_interrupts = 0;
    }
    return;
}

int main(void) {
    SPI1_Init(); // initializd SPI1
    UART2_Init(); // initialize UART2
    IEC1bits.U2RXIE = 1; // Abilita l'interrupt per la ricezione UART2
    U2STAbits.URXISEL = 3; // Set URXISEL to 11
    IEC0bits.INT0IE = 1; // enable INT0 interrupt botton S5
    IEC1bits.INT1IE = 1; //enable INT0 interrupt botton S6
    IEC0bits.T1IE = 1; //enable T1IE interrupt
    TRISDbits.TRISD0 = 1; // set the button S5 as input
       
    initBuffer();
    
    //char receivedChar;
    // STARTING THE LCD
    tmr_wait_ms(TIMER3,1000);
    
    // SET the TIMERS
    tmr_setup_period(TIMER1, 20); // Timer for the control bouncing of the button S5
    tmr_setup_period(TIMER2, 7); // Timer for the ALGORITHM
    tmr_setup_period(TIMER4, 10); // Timer fo the wait period in the MAIN loop
    
    initFunctionSecondRow();
    convertNumberToString(number_readings); 
    setCursorPositionFirstRow(0x80);
    // STARTING THE LCD
    tmr_wait_ms(TIMER3,1000);
    tmr_setup_period(TIMER3, 1); // Timer for moving the cursor from and to the first and second line of the LCD
   
    while (1) {
        algorithm();
        if(U2STAbits.URXDA == 1){   
            IEC1bits.U2RXIE = 0; // Disattiva l'interrupt per la ricezione UART2
            // Leggi il dato dal registro di ricezione UART2.
            char receivedData = U2RXREG;
            push_main(receivedData);   
            IEC1bits.U2RXIE = 1; // Abilita l'interrupt per la ricezione UART2
        }
        // Check if there are characters in the buffer
        pull();
        
        // Check the interrupts flag
        checkFlagsInterrupt();
        
        IFS0bits.T1IF = 0; // Reset the flag
        T1CONbits.TON = 1; // Starts the timer
        tmr_wait_period(TIMER1);
    }
}
