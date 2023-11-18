/*
 * Claudio Demaria s5433737
 * Gianluca Galvagni s5521188
 * 
 * DSPIC30F4011 Configuration Bit Settings
 * 'C' source line config statements
 * FOSC
 */ 
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
#define TIMER1 1                         // We use 4 timers
#define TIMER2 2
#define TIMER3 3
#define TIMER4 4
#define SIZE_OF_BUFFER 16                // Size of the circular buffer

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
void pull();
void checkFlagsInterrupt();
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt();
void __attribute__((__interrupt__, __auto_psv__)) _INT0Interrupt();
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt();
void __attribute__ ((__interrupt__, __auto_psv__)) _T1Interrupt();

//GLOBAL VARIABLES
int number_readings = 0;               // global variable for counting all the printed value
int number_first_row = 0;              // global variable for counting the printed value on the first raw
int position_first_row[16] = {0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x8B,0x8C,0x8D,0x8E,0x8F}; // Indexes for the second row of the LCD
int flags_interrupts = 0;              // interrupt flag for buttons S5 and S6

// TIMER FUNCTIONS
void tmr_setup_period(int timer, int ms){
    /*
     * Configure the prescaler and the PR (Period Register) values as follows:
     * With Fosc (oscillator frequency) set to 737,280 Hz, the resulting Fcy (instruction cycle frequency) is calculated as Fosc / 4, 
     * yielding 184,320 clock cycles per second. However, for a duration of 0.1 seconds, 
     * this would lead to 18,432 clock steps—a value exceeding the 16-bit register limit of 65,535.
     * To address this, a prescaler is set at 1:8, reducing the clock steps to 23,040 (184,320 / 8), 
     * which fits within the constraints of a 16-bit register.
     */ 
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
        T2CONbits.TCKPS = count; 
    }
    if (timer == TIMER3){
        TMR3 = 0.0;
        PR3 = clock_steps; 
        T3CONbits.TCKPS = count; 
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

void tmr_wait_ms(int timer, int ms) {
    tmr_setup_period(timer, ms);

    // Start the timer based on the specified timer number
    switch (timer) {
        case TIMER3:
            IFS0bits.T3IF = 0;  // Reset TIMER3 interrupt flag
            T3CONbits.TON = 1;  // Start TIMER3
            break;
        // Add cases for other timers if needed

        // Default case for unsupported timers
        default:
            // Handle unsupported timer case, if necessary
            break;
    }

    tmr_wait_period(timer);
}

void algorithm() {
    IFS0bits.T2IF = 0;      // Reset TIMER2 interrupt flag
    T2CONbits.TON = 1;      // Start TIMER2
    tmr_wait_period(TIMER2);
}

// INIT FUNCTIONS
void SPI1_Init() {
    // Function to initialize SPI1

    SPI1CONbits.MSTEN = 1;   // Set as master mode
    SPI1CONbits.MODE16 = 0;  // Set data mode to 8-bit
    SPI1CONbits.PPRE = 3;    // Set primary prescaler to 1:1
    SPI1CONbits.SPRE = 3;    // Set secondary prescaler to 5:1
    SPI1STATbits.SPIEN = 1;  // Enable SPI module
}

void UART2_Init() {
    // Function to initialize UART2

    U2MODE = 0;                 // Clear the mode register
    U2STA = 0;                  // Clear the status and control register

    // Set Baud Rate Generator value for a specific baud rate (e.g., 9600 bps)
    // Formula: (Fcy / (16 * Baud Rate)) - 1
    U2BRG = 11;                 // Example for 9600 bps with a 7.3728 MHz clock

    U2MODEbits.UARTEN = 1;      // Enable UART
    U2STAbits.UTXEN = 1;        // Enable UART transmitter
}

void initFunctionSecondRow() {
    /*
     * Function to initialize the second row: prints the 'Char Recv:' string and holds on it.
     * Subsequently, it will only print the updated current number of readings from UART2.
     */
    
    setCursorPositionSecondRow();

    char initSecondRow[11] = {'C','h', 'a', 'r' ,' ', 'R', 'e', 'c', 'v', ':', ' '}; // Start from column 10 or 11

    int i = 0;
    while (i < 11) {
        while (SPI1STATbits.SPITBF == 1);  // Wait until the SPI transmit buffer is not full
        SPI1BUF = initSecondRow[i];       // Transmit the character to SPI buffer
        i++;
    }
}

// SETTING CURSOR FUNCTIONS
void setCursorPositionFirstRow(int position) {
    // Function for setting the position of the cursor when printing on the first row

    IFS0bits.T3IF = 0;                // Wait for the cursor's movement and reset the flag
    T3CONbits.TON = 1;                // Start the timer
    tmr_wait_period(TIMER3);

    while(SPI1STATbits.SPITBF == 1);  // Wait until the SPI transmit buffer is not full
    SPI1BUF = position;               // Send a control command to set the cursor position
}

void setCursorPositionSecondRow() {
    // Function for setting the position of the cursor when printing on the second row

    while(SPI1STATbits.SPITBF == 1);  // Wait until the SPI transmit buffer is not full
    SPI1BUF = 0xC0;                    // Send a control command to set the cursor position

    IFS0bits.T3IF = 0;                 // Wait for the cursor's movement and reset the flag
    T3CONbits.TON = 1;                 // Start the timer
    tmr_wait_period(TIMER3);
}

// PRINTING/CLEANING FUNCTIONS
void cleaningFirstRow() {
    // Function for cleaning the first row

    char cleaner = ' ';                 // Character used for cleaning
    setCursorPositionFirstRow(0x80);    // Set cursor position to the beginning of the first row

    int i = 0;
    while (i < 16) {
        while (SPI1STATbits.SPITBF == 1);   // Wait until the SPI transmit buffer is not full
        SPI1BUF = cleaner;                 // Send the cleaner character to clear the display
        i++;
    }
}

void printFunctionFirstRow(char receivedChar) {
    // Function for printing the character received from UART2 (passed as a parameter)

    while (SPI1STATbits.SPITBF == 1);   // Wait until the SPI transmit buffer is not full
    SPI1BUF = receivedChar;             // Send the received character to the display

    number_first_row++;                 // Increment the character count on the first row

    if (receivedChar == '\r' || receivedChar == '\n') {
        // If a special character is received, clean the first row
        cleaningFirstRow();
        number_first_row = 0;
        setCursorPositionFirstRow(0x80);  // Set cursor position to the beginning of the first row
    }

    if (number_first_row == 16) {
        // If the first row is full, clean it
        cleaningFirstRow();
        number_first_row = 0;
        setCursorPositionFirstRow(0x80);  // Set cursor position to the beginning of the first row
    }
}

void printFunctionSecondRow(char printed_value[]) {
    /*
     * Function for printing the current number of received chars from UART2:
     * Printing starts from the 0xCA position because we keep the 'Char Recv:' string before that.
     */  
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0xCA;

    IFS0bits.T3IF = 0;  // Wait for the cursor to move and reset the flag
    T3CONbits.TON = 1;  // Start the timer
    tmr_wait_period(TIMER3);  // Wait for a specific period (assuming tmr_wait_period is defined elsewhere)

    int i = 0;
    while (1) {
        while (SPI1STATbits.SPITBF == 1);
        SPI1BUF = printed_value[i];

        // Check if the end of the string is reached
        if (printed_value[i] == '\0') {
            return;
        }
        i++;
    }
}

void convertNumberToString(int count) { 
    /*
     * Function for converting an integer (the count of the number of received chars from UART2) to a char 
     * and then print it on the second row
     */  
    char clr[19] = {' ', ' ', ' ',' ', ' ', ' '}; // Clean the positions where we'll print
    printFunctionSecondRow(clr); 

    char str[10];
    sprintf(str, "%d", count);                    // Convert int to char
    printFunctionSecondRow(str);                  // Print
}

// INTERRUPTS
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt
    () {
    /*
     * Interrupt function of the UART2: when the buffer is full, execute it.
     * Read the buffer and push the data inside the circular buffer
     */ 
    IFS1bits.U2RXIF = 0;                          // Reset the flag
    if(U2STAbits.URXDA == 1){                     // If there are data to be read
        char receivedData = U2RXREG;              // Read
        push(receivedData);                       // Push
    }    
}

void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt(){
    /*
     * Interrupt function for the S6 button: Sets the interrupt flag to trigger corresponding operations.
     */ 
    IFS1bits.INT1IF = 0;  // Reset INT1 interrupt flag
    flags_interrupts = 2;  // Set the interrupt flag value
}

void __attribute__((__interrupt__, __auto_psv__)) _INT0Interrupt(){
    /*
     * Interrupt function for button S5 (INT0): Resets the interrupt flag, 
     * disables INT0 interrupt, enables Timer1 (T1IE) interrupt, resets T1IF flag,
     * clears Timer1 register, and starts Timer1.
     */ 
    IFS0bits.INT0IF = 0;    // Reset INT0 interrupt flag 
    IEC0bits.INT0IE = 0;    // Disable INT0 interrupt for button S5
    IEC0bits.T1IE = 1;      // Enable Timer1 (T1IE) interrupt
    IFS0bits.T1IF = 0;      // Reset Timer1 interrupt flag
    TMR1 = 0;               // Clear Timer1 'counter' register
    T1CONbits.TON = 1;      // Start Timer1
}

void __attribute__ ((__interrupt__, __auto_psv__)) _T1Interrupt(){
    /*
     * Timer1 (T1) interrupt function: Resets the Timer1 interrupt flag,
     * disables Timer1 interrupt (T1IE), enables INT0 interrupt for button S5,
     * and checks if button S5 is already pressed, updating the interrupt flag accordingly.
     */ 
    IFS0bits.T1IF = 0;      // Reset Timer1 interrupt flag 
    IEC0bits.T1IE = 0;      // Disable Timer1 interrupt
    IEC0bits.INT0IE = 1;    // Enable INT0 interrupt for button S5

    // Check if button S5 is already pressed
    if(PORTEbits.RE8 == 1){
        flags_interrupts = 1;  // Set the interrupt flag value
    }
}

// CIRCULAR BUFFER
struct circular_buffer {
    // Struct that contains only the usefull information that we need from the buffer
    char buffer[SIZE_OF_BUFFER];                // Buffer
    int bufferLength;                           // Different from the SIZE_OF_BUFFER, it's 'writeIndex - readIndex', the number of bytes that we still have to read
    int readIndex;
    int writeIndex;
};

struct circular_buffer cb;                      // Our circular buffer

void initBuffer(){
    // Function to initialize the circular buffer
    cb.bufferLength = 0;
    cb.readIndex = 0;
    cb.writeIndex = 0;
}

void push(char receivedChar) {
    /*
     * Pushes characters received from UART into the circular buffer for writing.
     * If the circular buffer overflows, characters will be lost.
     */

    if (cb.bufferLength == SIZE_OF_BUFFER) {
        // Buffer is full, cannot push more characters
        return;
    } 
    else {
        // Push the character into the buffer
        cb.buffer[cb.writeIndex] = receivedChar;
        cb.bufferLength++;
        cb.writeIndex++;

        // Wrap around if the writeIndex exceeds the buffer size
        if (cb.writeIndex == SIZE_OF_BUFFER) {
            cb.writeIndex = 0;
        }
    }
}

void pull() {
    /*
     * Takes out all characters from the circular buffer and sends them to 'printFunctionFirstRow',
     * which will print them. When the circular buffer is empty, and the UART is not sending characters,
     * 'convertNumberToString' writes the number of characters printed on the second row of the LCD.
     */

    while (cb.bufferLength > 0) {
        if (cb.bufferLength == 0) {
            // Buffer is empty
            return;
        } 
        else {
            char receivedChar = cb.buffer[cb.readIndex];
            cb.bufferLength--;
            cb.readIndex++;
            number_readings++;

            // Wrap around if the readIndex exceeds the buffer size
            if (cb.readIndex == SIZE_OF_BUFFER) {
                cb.readIndex = 0;
            }

            printFunctionFirstRow(receivedChar);

            // Check if the buffer is empty and UART is not sending characters
            if (cb.bufferLength == 0 && U2STAbits.URXDA == 0) {
                convertNumberToString(number_readings);
                setCursorPositionFirstRow(position_first_row[number_first_row]);
            }
        }
    }
}

// INTERUPT FLAGS
void checkFlagsInterrupt() {
    /*
     * Manages interrupt flags. 
     * If flag = 1, sends the number of characters received to UART; 
     * if flag = 2, clears the LCD's first row and sets the number of characters received to zero.
     */

    if (flags_interrupts == 1) {
        while (U2STAbits.UTXBF);  // Wait for the transmit buffer to be empty
        U2TXREG = number_readings;  // Send the number of characters received
        flags_interrupts = 0;
    }

    if (flags_interrupts == 2) {
        cleaningFirstRow();
        number_readings = 0;
        number_first_row = 0;
        convertNumberToString(number_readings);
        setCursorPositionFirstRow(0x80);
        flags_interrupts = 0;
    }

    return;
}

// MAIN FUNCTION
int main(void) {   
    // Initialization
    SPI1_Init();            // Initialize SPI1
    UART2_Init();           // Initialize UART2
    IEC1bits.U2RXIE = 1;    // Enable interrupt for UART2 reception
    U2STAbits.URXISEL = 3;  // Set URXISEL to 11
    TRISDbits.TRISD0 = 1;   // Set button S5 as input
    TRISDbits.TRISD1 = 1;   // Set button S6 as input
    IEC0bits.INT0IE = 1;    // Enable INT0 interrupt for button S5
    IEC1bits.INT1IE = 1;    // Enable INT1 interrupt for button S6
    IEC0bits.T1IE = 1;      // Enable Timer1 (T1IE) interrupt
    
    // Circular buffer initialization
    initBuffer();
   
    // Set up timers
    tmr_setup_period(TIMER1, 20);  // Timer for controlling button S5 bouncing
    tmr_setup_period(TIMER2, 7);   // Timer for the algorithm
    tmr_setup_period(TIMER4, 10);  // Timer for the wait period in the main loop
    
    // Starting the LCD
    tmr_wait_ms(TIMER3, 1000);
    tmr_setup_period(TIMER3, 1);   // Timer for moving the cursor between the first and second lines of the LCD
    initFunctionSecondRow();
    convertNumberToString(number_readings); 
    setCursorPositionFirstRow(0x80);
    
    while (1) {
        algorithm();
        
        // Check if there are characters in UART2 buffer
        if (U2STAbits.URXDA == 1) {   
            IEC1bits.U2RXIE = 0;         // Disable interrupt for UART2 reception
            char receivedData = U2RXREG; // Read from UART2
            push(receivedData);   
            IEC1bits.U2RXIE = 1;        // Enable UART2 reception interrupt
        }
        
        pull();                         // Check if there are characters in the buffer
        checkFlagsInterrupt();          // Check the interrupt flags
        
        IFS1bits.T4IF = 0;              // Reset Timer4 interrupt flag
        T4CONbits.TON = 1;              // Start Timer4
        tmr_wait_period(TIMER4);
    }
}
