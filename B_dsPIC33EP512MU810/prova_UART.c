#include "xc.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"

#define FOSC 144000000.0 //Hz Clock breadboard
#define REG_SXT_BIT 65535.0 // MAX 16 bit register
#define TIMER1 1
#define TIMER2 2
#define TIMER3 3
#define TIMER4 4
#define TIMER5 5
#define STATE_WAIT 0
#define STATE_MOTION 1
#define SIZE_OF_BUFFER 7
#define SIZE_OF_MESSAGE_BUFFER 54

#define STATE_DOLLAR  (1) // we discard everything until a dollar is found
#define STATE_TYPE    (2) // we are reading the type of msg until a comma is found
#define STATE_PAYLOAD (3) // we read the payload until an asterix is found
#define NEW_MESSAGE (1) // new message received and parsed completely
#define NO_MESSAGE (0) // no new messages


// Function prototypes
void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
void tmr_wait_ms(int timer, int ms);
// interrupt functions
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt();
void __attribute__ ((__interrupt__, __auto_psv__)) _T1Interrupt();
void __attribute__ ((__interrupt__, __auto_psv__)) _T3Interrupt();
float threshold_calculation(float y_cm);
// ligtts
void leds_initialization();
void lights_motion(float sg, float yr);
void lights_off();
// buttons
void bottoms_initialization();
// UART
void UART_initialization();
// Circular buffer
void initBuffer();
void push(char receivedChar);
void pull();

int mainState = STATE_WAIT; // 0 = wait, 1 = move
int counterBlink = 0;

// CIRCULAR BUFFER
struct circular_buffer {
    // Struct that contains only the usefull information that we need from the buffer
    char buffer[SIZE_OF_BUFFER];                // Buffer
    int bufferLength;                           // Different from the SIZE_OF_BUFFER, it's 'writeIndex - readIndex', the number of bytes that we still have to read
    int readIndex;
    int writeIndex;
};

typedef struct { 
	int state;
	char msg_type[6]; // type is 5 chars + string terminator
	char msg_payload[100];  // assume payload cannot be longer than 100 chars
	int index_type;
	int index_payload;
} parser_state;

int parse_byte(parser_state* ps, char byte);

// Structure definition for the thresholds to be set and considered for PWM.
typedef struct {
    int minth;
    int maxth;
} threshold_data;

volatile parser_state ps;
struct circular_buffer cb;                      // Our circular buffer
volatile threshold_data sdata;

void tmr_setup_period(int timer, int ms){ // Set the prescaler and the PR value
// Fosc = 144000000 Hz -> Fcy = Fosc / 2 = 72000000 number of clocks in one second so in 0.1 secon there would be 7200000 clocks steps
// this is too high to be put in a 16 bit register (max 65535)
// we set the millisec value and the max value for that breadboard is more or less 0.582 (<582ms) seconds
// If we set ms = 500 we have 144000000/2 * 500/1000 = 36000000 clock steps
// this is too high to be put in a 16 bit register (max 65535)
// If we set a prescaler of 1:8 we have 36000000/8 = 4500000 clock steps
// this is too high to be put in a 16 bit register (max 65535)
// If then we set a prescaler of 1:64 we have 36000000/64 = 562500 clock steps
// this is too high to be put in a 16 bit register (max 65535)
// If then we set a prescaler of 1:256 we have 36000000/256 = 140625 clock steps
// this is ok to be put in a 16 bit register (max 65535)

    double Fcy = FOSC / 2.0;
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
        //T1CONbits.TCS = 0;
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
        T4CONbits.TCKPS = count; 
    }
    if (timer == TIMER5){
        TMR5 = 0.0;
        PR5 = clock_steps; 
        T5CONbits.TCKPS = count; 
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
    if (timer == TIMER3){
      while(IFS0bits.T3IF == 0){
        // Wait until the flag is high -> The timer finished to count
        }  
    }
    if (timer == TIMER4){
      while(IFS1bits.T4IF == 0){
        // Wait until the flag is high -> The timer finished to count
        }  
    }
    if (timer == TIMER5){
      while(IFS1bits.T5IF == 0){
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
    if (timer == TIMER3){
        IFS0bits.T3IF = 0; // Reset the flag
        T3CONbits.TON = 1; // Starts the timer
    }
    if (timer == TIMER4){
        IFS1bits.T4IF = 0; // Reset the flag
        T4CONbits.TON = 1; // Starts the timer
    }
    if (timer == TIMER5){
        IFS1bits.T5IF = 0; // Reset the flag
        T5CONbits.TON = 1; // Starts the timer
    }
    tmr_wait_period(timer);
}

//////////////////////// INITIALIZATION FUNCTIONS ////////////////////////
void bottoms_initialization(){
    // Pin configuration for the buttons
    RPINR0bits.INT1R = 0x58 ; // Set pin RP24 as INT1 (button E8)
    //RPINR1bits.INT2R = 0x59 ; // Set pin RP25 as INT2 (button E9)
    INTCON2bits.GIE = 1; // Enable global interrupt
    INTCON2bits.INT1EP = 1; // Set INT1 interrupt on falling edge
    //INTCON2bits.INT2EP = 1; // Set INT2 interrupt on falling edge
    // Button E8 and E9 configuration
    TRISEbits.TRISE8 = 1; // Set pin RE8 as input
    //TRISEbits.TRISE9 = 1; // Set pin RE9 as input   
    // Buttons Interrupt configuration
    IFS1bits.INT1IF = 0; // Reset INT1 interrupt flag
    //IFS1bits.INT2IF = 0; // Reset INT2 interrupt flag
    // eneable INT1 and INT2 interrupts for E8 and E9 buttons
    IEC1bits.INT1IE = 1; // Enable INT1 interrupt for button E8
    //IEC1bits.INT2IE = 1; // Enable INT2 interrupt for button E9
    // Configuration for timer interrupts for the button E8
    tmr_setup_period(TIMER1, 100); // set the timer to press the button E8 for 100ms and then stop the motion
    IEC0bits.T1IE = 1; // Enable Timer1 (T1IE) interrupt
}

void leds_initialization(){
    // Configuration for the leds
    TRISAbits.TRISA0 = 0; // Set pin RA0 as output
    TRISGbits.TRISG9 = 0; // Set pin RG9 as output
    TRISBbits.TRISB8 = 0; // Set pin RB8 as output -> left side lights
    TRISFbits.TRISF1 = 0; // Set pin RF1 as output -> right side lights
    TRISFbits.TRISF0 = 0; // Set pin RF0 as output -> breaks lights
    TRISGbits.TRISG1 = 0; // Set pin RG1 as output -> low intensity lights
    TRISAbits.TRISA7 = 0; // Set pin RA7 as output -> beam lights
    // Set the lights off
    lights_off(); 
}

void UART_initialization(){
    // Configuration for UART
    U2MODE = 0; // Clear UART2 mode register
    U2STA = 0; // Clear UART2 status register
    U2BRG = 157; // 28800 baud rate

    U2MODEbits.UARTEN = 1; // Enable UART2
    U2STAbits.UTXEN = 1; // Enable UART2 transmission
    U2STAbits.URXISEL = 3; // Interrup when the buffer is full
    IEC1bits.U2RXIE = 1; // Enable UART2 interrupt
    U2STAbits.UTXISEL0 = 0; // Interrupt when the buffer is empty
    U2STAbits.UTXISEL1 = 0; // Interrupt when the buffer is empty

    // UART2 TX and RX pins
    RPOR0bits.RP64R = 0x03; // Set pin RP64 as U2TX
    RPINR19bits.U2RXR = 0x4B; // Set pin RP75 as U2RX
}

// We won't delve deeper in the explaination of the functions "parse_byte", "extract_integer",
// "next_value" and "parse_pcth", because they have been given to us already made.
int parse_byte(parser_state* ps, char byte) {
    switch (ps->state) {
        case STATE_DOLLAR:
            if (byte == '$') {
                ps->state = STATE_TYPE;
                ps->index_type = 0;
            }
            break;
        case STATE_TYPE:
            if (byte == ',') {
                ps->state = STATE_PAYLOAD;
                ps->msg_type[ps->index_type] = '\0';
                ps->index_payload = 0; // initialize properly the index
            } else if (ps->index_type == 6) { // error! 
                ps->state = STATE_DOLLAR;
                ps->index_type = 0;
			} else if (byte == '*') {
				ps->state = STATE_DOLLAR; // get ready for a new message
                ps->msg_type[ps->index_type] = '\0';
				ps->msg_payload[0] = '\0'; // no payload
                return NEW_MESSAGE;
            } else {
                ps->msg_type[ps->index_type] = byte; // ok!
                ps->index_type++; // increment for the next time;
            }
            break;
        case STATE_PAYLOAD:
            if (byte == '*') {
                ps->state = STATE_DOLLAR; // get ready for a new message
                ps->msg_payload[ps->index_payload] = '\0';
                return NEW_MESSAGE;
            } else if (ps->index_payload == 100) { // error
                ps->state = STATE_DOLLAR;
                ps->index_payload = 0;
            } else {
                ps->msg_payload[ps->index_payload] = byte; // ok!
                ps->index_payload++; // increment for the next time;
            }
            break;
    }
    return NO_MESSAGE;
}

int extract_integer(const char* str) { 
  int i = 0, number = 0, sign = 1;

  if (str[i] == '-') {
    sign = -1;
    i++;
  }
  else if (str[i] == '+') {
    sign = 1;
    i++;
  }
  while (str[i] != ',' && str[i] != '\0'){
    number = number * 10;
    number += str[i] - '0';
    i++;
  }
  return sign * number;
}

int next_value(const char* msg, int i) {
  while (msg[i] != ',' && msg[i] != '\0') {
    i++;
  }
  if (msg[i] == ',') {
    i++;
  }
  return i;
}

void parse_pcth(const char* msg, volatile threshold_data* sdata){
  int i = 0;
  sdata->minth = extract_integer(msg);
  i = next_value(msg, i);
  sdata->maxth = extract_integer(msg + i);
}

///////////////////////// INTERRUPT FUNCTIONS /////////////////////////
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt(){
    /*
     * Interrupt function for button E8 (INT1): Resets the interrupt flag, 
     * disables INT1 interrupt, enables Timer1 (T1IE) interrupt, resets T1IF flag,
     * clears Timer1 register, and starts Timer1.
     */ 
    IFS1bits.INT1IF = 0;    // Reset INT1 interrupt flag 
    IEC1bits.INT1IE = 0;    // Disable INT1 interrupt for button S5
    IEC0bits.T1IE = 1;      // Enable Timer1 (T1IE) interrupt
    IFS0bits.T1IF = 0;      // Reset Timer1 interrupt flag
    TMR1 = 0;               // Clear Timer1 'counter' register
    T1CONbits.TON = 1;      // Start Timer1
}

void __attribute__ ((__interrupt__, __auto_psv__)) _T1Interrupt(){
    /*
     * Timer1 (T1) interrupt function: Resets the Timer1 interrupt flag,
     * disables Timer1 interrupt (T1IE), enables INT1 interrupt for button E8,
     * and checks if button E8 is pressed. If it is pressed, it stops the motion.
     */ 
    IFS0bits.T1IF = 0;      // Reset Timer1 interrupt flag 
    IEC0bits.T1IE = 0;      // Disable Timer1 interrupt
    IFS1bits.INT1IF = 0;    // Reset INT1 interrupt flag
    IEC1bits.INT1IE = 1;    // Enable INT1 interrupt for button S5

    // Check if button E8 is already pressed
    if(PORTEbits.RE8 == 0){
        if (mainState == STATE_WAIT){
            mainState = STATE_MOTION;
        }
        else if (mainState == STATE_MOTION){
            mainState = STATE_WAIT;
        }
        // stop the timer3
        T3CONbits.TON = 0; 
        TMR3 = 0.0;
        counterBlink = 0;
        // turn off the lights
        lights_off();
    }
}

void __attribute__ ((__interrupt__, __auto_psv__)) _T3Interrupt(){
    /*
    * Timer3 (T3) interrupt function: Resets the Timer3 interrupt flag,
    * implements the variable 'counterBlink' because we cannot have a timer with a period of 1 second,
    * so we wait two times the interrupt period (500ms) to have a period of 1 second. 
    * After that, we blink the lights at 1Hz.
    */
    IFS0bits.T3IF = 0;      // Reset Timer3 interrupt flag
    TMR3 = 0.0;
    counterBlink += 1;
    if (counterBlink == 5 && mainState == STATE_WAIT){
        // blink the lights at 1Hz
        LATGbits.LATG9 = !LATGbits.LATG9;
        LATAbits.LATA0 = !LATAbits.LATA0;
        LATBbits.LATB8 = !LATBbits.LATB8;
        LATFbits.LATF1 = !LATFbits.LATF1;
        counterBlink = 0;
        return;
    }
    else if (counterBlink == 5 && mainState == STATE_MOTION){
        // blink the right side lights at 1Hz
        LATFbits.LATF1 = !LATFbits.LATF1;
        counterBlink = 0;
        return;
    }
    T3CONbits.TON = 1; // start the timer
}



void lights_motion(float sg, float yr){
    // Set the lights
    if(sg >= 50.0){
        LATFbits.LATF0 = 0; 
        LATGbits.LATG1 = 0;
        LATAbits.LATA7 = 1; // beam lights on
    } else {
        LATFbits.LATF0 = 1; // breaks lights on
        LATGbits.LATG1 = 1; // low intensity lights on
        LATAbits.LATA7 = 0; 
    }

    if (yr > 15.0){
        // blink the left side lights at 1Hz
        if (counterBlink == 0){
            T3CONbits.TON = 1;
        }
    }
    else {
        // stop the timer3
        T3CONbits.TON = 0;
        TMR3 = 0.0;
        counterBlink = 0;
        // turn off the left and right side lights
        LATBbits.LATB8 = 0; 
        LATFbits.LATF1 = 0; 
    }
}

void lights_off(){
    LATAbits.LATA0 = 0; 
    LATGbits.LATG9 = 0; 
    LATBbits.LATB8 = 0; 
    LATFbits.LATF1 = 0; 
    LATFbits.LATF0 = 0; 
    LATGbits.LATG1 = 0; 
    LATAbits.LATA7 = 0;
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
            IEC1bits.U2RXIE = 0; // Disable UART2 Receiver Interrupt to avoid problems with shared variables  
            char receivedChar = cb.buffer[cb.readIndex];
            cb.bufferLength--;
            cb.readIndex++;

            // Wrap around if the readIndex exceeds the buffer size
            if (cb.readIndex == SIZE_OF_BUFFER) {
                cb.readIndex = 0;
            }
            int message = parse_byte(&ps, receivedChar); // Parse the byte to get the message type
            IEC1bits.U2RXIE = 1; // Enable UART2 Receiver Interrupt
            
            if (message == NEW_MESSAGE) { // If we have a new message, we acquire the payload into sdata.minth and maxth
                if (ps.msg_type[0] == 'P' && ps.msg_type[1] == 'C' && ps.msg_type[2] == 'T' && ps.msg_type[3] == 'H' && ps.msg_type[4] == '\0'){
                    parse_pcth(ps.msg_payload, &sdata);
                }
            }
        }
    }
}

// INTERRUPT UART
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt
    () {
    /*
     * Interrupt function of the UART2: when the buffer is full, execute it.
     * Read the buffer and push the data inside the circular buffer
     */ 
    IFS1bits.U2RXIF = 0;                          // Reset the flag
    if(U2STAbits.URXDA == 1){                     // If there are data to be read
        //IEC1bits.U2RXIE = 0;                      // Disable interrupt for UART2 reception
        char receivedData = U2RXREG;              // Read
        push(receivedData);                       // Push
        //IEC1bits.U2RXIE = 1;                       // Push
    }    
}

// Function to calculate the threshold based on our empirical data..
float threshold_calculation(float y_cm){
    float threshold = (float) ((0.016 * y_cm) - 0.025);
    return threshold;
}

int main(void){
    ANSELA=ANSELB=ANSELC=ANSELD=ANSELE=ANSELG=0x0000; //MANDATORY 

    // initializations
    leds_initialization();
    bottoms_initialization();   
    UART_initialization();

    // set the timer for the main at 1kHz
    tmr_setup_period(TIMER2, 1); // set the timer to 1ms
    // set the timer for the blink at 1Hz (200ms x 5 = 1s)
    tmr_setup_period(TIMER3, 200); // set the timer to 200ms
    IEC0bits.T3IE = 1; // Enable Timer3 (T3IE) interrupt

    float distance = 0.0;

    // Circular buffer initialization:
    cb.writeIndex = 0;
    cb.readIndex = 0;
    cb.bufferLength = 0;
    
    // Parser state initialization:
    ps.state = STATE_DOLLAR; 
    ps.index_type = 0; 
    ps.index_payload = 0;
    
    // Threshold data initialization:
    sdata.minth = 15; 
    sdata.maxth = 38;
    
    while (1)
    {
        // Check if there are characters in UART2 buffer
        if (U2STAbits.URXDA == 1) {   
            IEC1bits.U2RXIE = 0;         // Disable interrupt for UART2 reception
            char receivedData = U2RXREG; // Read from UART2
            push(receivedData);   
            IEC1bits.U2RXIE = 1;         // Enable UART2 reception interrupt
        }
        pull();

        IFS0bits.T2IF = 0;        
        T2CONbits.TON = 1;  
        tmr_wait_period(TIMER2);
    }
    return 0;
}