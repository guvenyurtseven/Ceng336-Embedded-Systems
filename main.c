#ifndef PRAGMAS_H
#define	PRAGMAS_H
#ifdef	__cplusplus
extern "C" {
#endif
#include <xc.h>
#include <p18cxxx.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define _XTAL_FREQ   40000000
#pragma config  OSC = HSPLL, FCMEN = OFF, IESO = OFF
#pragma config  PWRT = OFF, BOREN = OFF, BORV = 3
#pragma config  WDT = OFF, WDTPS = 32768
#pragma config  MODE = MC, ADDRBW = ADDR20BIT, DATABW = DATA16BIT, WAIT = OFF
#pragma config  CCP2MX = PORTC, ECCPMX = PORTE, LPT1OSC = OFF, MCLRE = ON
#pragma config  STVREN = ON, LVP = OFF, BBSIZ = BB2K, XINST = OFF
#pragma config  CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF, CP4 = OFF, CP5 = OFF
#pragma config  CP6 = OFF, CP7 = OFF
#pragma config  CPB = OFF, CPD = OFF
#pragma config  WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF, WRT4 = OFF
#pragma config  WRT5 = OFF, WRT6 = OFF, WRT7 = OFF
#pragma config  WRTC = OFF, WRTB = OFF, WRTD = OFF
#pragma config  EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF, EBTR4 = OFF
#pragma config  EBTR5 = OFF, EBTR6 = OFF, EBTR7 = OFF
#pragma config  EBTRB = OFF
#pragma config DEBUG = ON
#endif	/* PRAGMAS_H */
// ======================================================//
//                     GLOBALS                           //
//                                                       //
//    const unsigned char Guven Yurtseven, 2449080       //
//    const unsigned char Ersin Sert, 2448819            //
//    const unsigned char Mehmet Ozgur Goksu, 2310043    //
//                                                       //
// ======================================================//

int TOTAL_EMPTY_SPACE = 40;     // Total number of empty parking spaces (initially 40)
int TOTAL_MONEY_EARNED = 0;     // Total money earned from parking fees
int message_queue_head=0;       // Head pointer for message queue (circular buffer)
int message_queue_tail=0;       // Tail pointer for message queue (circular buffer)
int plate_queue_tail=0;         // Tail pointer for plate queue (waiting cars)
int plate_queue_head=0;         // Head pointer for plate queue (waiting cars)
int message_count = 0;          // Current number of messages in queue
int plate_count = 0;            // Current number of plates in queue
char MessageQueue[16][12];      // Circular buffer for outgoing messages (16 messages, 12 chars each)
char PlateQueue[16][4];         // Circular buffer for waiting car plates (16 plates, 4 chars each)
char message_to_send[12];       // Buffer for current message being sent
char dequeued_plate[4];         // Buffer for plate number dequeued from waiting queue
int message_var=0;              // Flag indicating if there's a message ready to send
int output_due = 0;             // Flag indicating when 100ms output period is due
int counter_for_output = 0;     // Counter for 100ms output timing (20 * 5ms = 100ms)
char display_mode = 'm';        // Display mode: 'm' for money, 'e' for empty spaces
int end_flag = 0;               // Flag to terminate main loop when END command received
unsigned int five_ms_counter = 0; // Counter for 5ms timer interrupts (used for parking duration)
uint8_t adc_value   = 0;        // Current ADC value (0-3 representing levels A-D)
int TOTAL_RES_SPACE = 0;
int TOTAL_EMPTY_RES_SPACE = 0;// Total number of reserved parking spaces
int current_digit=0;            // Current digit being displayed on 7-segment (0=leftmost, 3=rightmost)
int time_to_adc=0;              // Counter for ADC sampling period (500ms)
int current_RB4_state=0;        // Current state of RB4 button
int previous_RB4_state=1;       // Previous state of RB4 button for edge detection
uint8_t segment_values[] = {
    0x3F,  // 0
    0x06,  // 1
    0x5B,  // 2
    0x4F,  // 3
    0x66,  // 4
    0x6D,  // 5
    0x7D,  // 6
    0x07,  // 7
    0x7F,  // 8
    0x6F,  // 9
    0x00   // void
};

#define PKT_MAX_SIZE 128 // Maximum packet size.                                                    
#define PKT_HEADER '$'  // Marker for start-of-packet $
#define PKT_END '#'    // Marker for end-of-packet as #
#define BUFSIZE 128


typedef enum { INBUF = 0, OUTBUF = 1 } buf_t;
uint8_t inbuf[BUFSIZE], outbuf[BUFSIZE];
uint8_t head[2] = {0,0}, tail[2] = {0,0};

// Inline functions for enabling/disabling USART interrupts
inline void disable_rxtx( void ) { PIE1bits.RC1IE = 0;PIE1bits.TX1IE = 0;}
inline void enable_rxtx( void )  { PIE1bits.RC1IE = 1;PIE1bits.TX1IE = 1;}

#pragma interrupt_level 2 // Prevents duplication of function
uint8_t buf_isempty( buf_t buf ) { return (head[buf] == tail[buf])?1:0; }

#pragma interrupt_level 2
void buf_push( uint8_t v, buf_t buf) {
    if (buf == INBUF) inbuf[head[buf]] = v;
    else outbuf[head[buf]] = v;
    head[buf]++;
    if (head[buf] == BUFSIZE) head[buf] = 0;
}

#pragma interrupt_level 2 // Prevents duplication of function
uint8_t buf_pop( buf_t buf ) {
    uint8_t v;
    if (buf_isempty(buf)) { 
        return 0; 
    } 
    else {
        if (buf == INBUF) v = inbuf[tail[buf]];
        else v = outbuf[tail[buf]];
        tail[buf]++;
        if (tail[buf] == BUFSIZE) tail[buf] = 0;
        return v;
    }
}
/**
 * Initialize message and plate queues
 * Clears all queue arrays and sets all elements to null terminators
 */
void init_queue(){
    for(int i = 0; i< 16; i++){
        for(int j = 0; j<12 ; j++){
            MessageQueue[i][j]='\0';
        }
        for(int k = 0; k<4; k++){ 
            PlateQueue[i][k]='\0';
        }
    }
}
/**
 * Add a message to the message queue (FIFO)
 * @param new: Pointer to 12-character message string to enqueue
 */
void message_enqueue(char * new){
    
    if (message_count == 16){return;}
    
    else{
        for( int i = 0 ; i<12 ; i++){                    
            MessageQueue[message_queue_tail][i]=new[i];
        }
        MessageQueue[message_queue_tail][11]='\0';
        message_queue_tail = (message_queue_tail+1)%16;  
        message_count++;
        return;
    }  
}
/**
 * Add a plate number to the plate queue (FIFO) for waiting cars
 * @param new: Pointer to 4-character plate string to enqueue
 */
void plate_enqueue(char * new){
    if (plate_count == 16){return;}
    else{
        for ( int i = 0 ; i<4 ; i++){                   // Copy each character of the plate
            PlateQueue[plate_queue_tail][i]=new[i];
        }
        PlateQueue[plate_queue_tail][3]='\0';
        plate_queue_tail = (plate_queue_tail+1)%16;      // Circular increment of tail pointer
        plate_count++;
        return;
    }
}

/**
 * Remove and retrieve a message from the message queue (FIFO)
 * Sets message_var flag to indicate if a message was successfully dequeued
 */
void message_dequeue(){
    if (message_count==0){              // Check if queue is empty
        message_var=0;                  // No message available
        return;
    }
    else
    {
        for(int i = 0 ; i<12 ; i++){
            message_to_send[i]=MessageQueue[message_queue_head][i];
            MessageQueue[message_queue_head][i]='\0';       // Clear the queue slot
        }
        message_queue_head = (message_queue_head+1) % 16;
        message_count--;
        message_var = 1;
        return;
    }
}
/**
 * Remove and retrieve a plate number from the plate queue (FIFO)
 * Used when a parking space becomes available for waiting cars
 */
void plate_dequeue(){
    if (plate_count==0){ 
        return;
    }
    else
    {
        for(int i = 0 ; i<4 ; i++){
            dequeued_plate[i]=PlateQueue[plate_queue_head][i];
            PlateQueue[plate_queue_head][i]='\0';
        }
        plate_queue_head = (plate_queue_head+1) % 16;
        plate_count--;
        return;
    }
}
/**
 * Parking slot structure definition
 * Contains all information needed to manage each parking space in the 4-level parking lot
 */
struct SLOT {
    char plate[4] ;         // Plate number of parked car (3 digits + null terminator)
    char high,low;          // High and low digits of parking spot number (01-10)
    unsigned int start_time; // Time when car started parking (in 5ms units)
    unsigned int is_empty;   // Flag: 1 if slot is empty, 0 if occupied
    unsigned int is_reserved; // Flag: 1 if slot is reserved via subscription, 0 if not
    char level;             // Parking level: 'A', 'B', 'C', or 'D'
};
/**
 * Initialize all parking slots with default values
 * Sets up 4 levels (A,B,C,D) with 10 slots each (01-10)
 */
static struct SLOT slot_list[40];
void init_slot(){
    for(int i = 0; i<40 ; i++){
        slot_list[i].is_empty=1;                // Initially all slots are empty
        slot_list[i].start_time=0;              // No parking time recorded
        slot_list[i].is_reserved=0;             // No slots initially reserved
        if (i < 9){
            slot_list[i].level = 'A';
            slot_list[i].high='0';
            slot_list[i].low='0'+((i%10)+1);
        }
        else if (i==9){
            slot_list[i].level = 'A';
            slot_list[i].high='1';
            slot_list[i].low='0';
        }
        else if (i < 19){
            slot_list[i].level = 'B';
            slot_list[i].high='0';
            slot_list[i].low='0'+((i%10)+1);
        }
        else if (i==19){
            slot_list[i].level = 'B';
            slot_list[i].high='1';
            slot_list[i].low='0';
        }
        else if (i < 29){
            slot_list[i].level = 'C';
            slot_list[i].high='0';
            slot_list[i].low='0'+((i%10)+1);
        }
        else if (i==29){
            slot_list[i].level = 'C';
            slot_list[i].high='1';
            slot_list[i].low='0';
        }
        else if (i < 39){
            slot_list[i].level = 'D';
            slot_list[i].high='0';
            slot_list[i].low='0'+((i%10)+1);
        }
        else if (i==39){
            slot_list[i].level = 'D';
            slot_list[i].high='1';
            slot_list[i].low='0';
        }
    }
}
/**
 * Initialize 7-segment display hardware
 * Sets up PORTH for digit selection and PORTJ for segment control
 */
void start_seven_segment(){
    LATH = 0x00; PORTH = 0x00; TRISH = 0x10;   // Clear PORTH output latches, set RH4 as input (for ADC), others as output
    LATJ = 0x00; PORTJ = 0x00; TRISJ = 0x00;   // Clear PORTJ output latches, set all PORTJ pins as output for segments
    return;
}
/**
 * Display a single digit on the 7-segment display
 * @param digit: Digit value (0-9) to display, 10 for blank
 */
void display_digit(int digit){
    switch (current_digit) {        // Select which digit position to activate
        case 0:
            PORTH = 0x08;  
            break;
        case 1:
            PORTH = 0x04; 
            break;
        case 2:
            PORTH = 0x02;  
            break;
        case 3:
            PORTH = 0x01; 
            break;
    }
    LATJ = segment_values[digit];   // Set segment pattern on PORTJ
    return;
}
/**
 * Display total money earned on 7-segment display
 * Multiplexes through 4 digits to show complete amount
 */
void show_total_money(){
    current_digit++;
    if(current_digit>=4){
        current_digit=0;
    }
    int money_into_digits[4];
    money_into_digits[0]=TOTAL_MONEY_EARNED%10;
    money_into_digits[1]=(TOTAL_MONEY_EARNED/10)%10;
    money_into_digits[2]=(TOTAL_MONEY_EARNED/100)%10;
    money_into_digits[3]=(TOTAL_MONEY_EARNED/1000)%10;
    display_digit(money_into_digits[current_digit]);
    return;
}
/**
 * USART receive interrupt service routine
 * Called when a byte is received via serial communication
 */
void receive_isr() {
    PIR1bits.RC1IF = 0;      // Acknowledge interrupt
    buf_push(RCREG1, INBUF); // Buffer incoming byte
}
/**
 * Enable global interrupts to start system operation
 */
void start_system() { 
    INTCONbits.GIE = 1; 
}
/**
 * Packet processing state machine for serial communication protocol
 * Handles the parsing of incoming serial packets with $ header and # terminator
 */
/* **** Packet task **** */
typedef enum {PKT_WAIT_HDR, PKT_GET_BODY, PKT_WAIT_ACK} pkt_state_t;
pkt_state_t pkt_state = PKT_WAIT_HDR;
uint8_t pkt_body[PKT_MAX_SIZE]; // Packet data
uint8_t pkt_bodysize;           // Size of the current packet
uint8_t pkt_valid = 0;          // Set to 1 when packet is received. Must be set to 0 once packet is processed

void packet_task() {
    disable_rxtx();                 // Disable interrupts during buffer access
    // Wait until new bytes arrive
    if (!buf_isempty(INBUF)) {
        uint8_t v;
        switch(pkt_state) {
        case PKT_WAIT_HDR:
            v = buf_pop(INBUF);
            if (v == PKT_HEADER) {
                // Packet header is encountered, retrieve the rest of the packet
                pkt_state = PKT_GET_BODY;
                pkt_bodysize = 0;
            }
            break;
        
        case PKT_GET_BODY:
            v = buf_pop(INBUF);
            if (v == PKT_END) {
                // End of packet is encountered, signal calc_task()
                pkt_state = PKT_WAIT_ACK;
                pkt_valid = 1;
            } 
            else 
                pkt_body[pkt_bodysize++] = v;
            break;
        case PKT_WAIT_ACK:
            if (pkt_valid == 0) {
                // Packet processing seems to be finished, continue monitoring
                pkt_state = PKT_WAIT_HDR;
            }
            break;
        }
    }
    enable_rxtx();
}

#define SPBRG_VAL (21)          // Baud rate generator value for 115200 bps at 40MHz
/**
 * Initialize USART1 for serial communication
 * Configures 115200 baud, 8-N-1 format for communication with Python simulator
 */
void init_serial() {
    // We will configure EUSART1 for 115200 baud
    // SYNC = 0, BRGH = 0, BRG16 = 1. 
    
    TXSTA1bits.TX9 = 0;    // no 9th bit
    TXSTA1bits.TXEN = 0;   // transmission is disabled for the time being
    TXSTA1bits.SYNC = 0;   // asynchronous transmission
    TXSTA1bits.BRGH = 0;   // low speed
    RCSTA1bits.SPEN = 1;   // enable serial port
    RCSTA1bits.RX9 = 0;    // no 9th bit
    RCSTA1bits.CREN = 1;   // continuous reception
    BAUDCON1bits.BRG16 = 1;// 16-bit transition

    SPBRGH1 = (SPBRG_VAL >> 8) & 0xff;
    SPBRG1 = SPBRG_VAL & 0xff;
}
/**
 * USART transmit interrupt service routine
 * Called when transmit buffer is ready for next byte
 */
/* ***** Serial output task **** */
void transmit_isr() {
    PIR1bits.TX1IF = 0;    // Acknowledge interrupt
    // If all bytes are transmitted, turn off transmission
    if (buf_isempty(OUTBUF)) {
        while (TXSTA1bits.TRMT == 0){}
        TXSTA1bits.TXEN = 0; 
    }
    // Otherwise, send next byte
    else TXREG1 = buf_pop(OUTBUF);
}

/* Output a string to the outgoing buffer */
// Push whole string, then kick off the first byte
void output_str(const char *str) {
    size_t len = strlen(str);
    for (size_t i = 0; i < len; i++) {
        disable_rxtx();
        buf_push(str[i], OUTBUF);
        enable_rxtx();
    }
}
/**
 * Serial output task state machine
 * Manages the transmission of buffered output data
 */
typedef enum {OUTPUT_INIT, OUTPUT_RUN} output_st_t;
output_st_t output_st = OUTPUT_INIT;
/* Output task function */
void output_task() {
    switch (output_st) {
    case OUTPUT_INIT:
        // Initial state - transition to running state
        output_st = OUTPUT_RUN;
        break;
    case OUTPUT_RUN:
        // Main operating state - check for pending output
        disable_rxtx();                         // Disable interrupts during buffer check
        // Start transmission if buffer has data and transmitter is idle
        if (!buf_isempty(OUTBUF)&& TXSTA1bits.TXEN == 0) { 
            // Send first byte and enable transmission (ISR handles remaining bytes)
            TXREG1 = buf_pop(OUTBUF);           // Send first byte to USART
            TXSTA1bits.TXEN = 1;                // Enable transmitter (triggers interrupt for subsequent bytes)
        }
        enable_rxtx();                          // Re-enable interrupts
        break;
    }
}


void init_interrupts() {
    enable_rxtx();                              // Enable USART receive and transmit interrupts
    INTCONbits.TMR0IE = 1;                      // Enable Timer0 overflow interrupt
    INTCONbits.PEIE = 1;                        // Enable peripheral interrupts
    PIE1bits.ADIE = 1;                          // Enable ADC conversion complete interrupt
    TRISBbits.TRISB4 = 1;                       // Set RB4 as input for button
    INTCON2bits.RBPU = 0;                       // Enable PORTB pull-up resistors
    uint8_t dummyRead = PORTB;                  // Read PORTB to clear mismatch condition
    INTCONbits.RBIE = 1;                        // Enable PORTB interrupt on change
    INTCONbits.RBIF = 0;                        // Clear PORTB interrupt flag
}
/**
 * Initialize I/O port directions and initial states
 * Configures pins for USART, 7-segment display, and button input
 */
void init_ports() {
    TRISCbits.TRISC6 = 0;                       // RC6 (TX1) as output for USART transmission
    TRISCbits.TRISC7 = 1;                       // RC7 (RX1) as input for USART reception
    TRISA = 0x00;                               // All PORTA pins as output
    TRISH = 0x10;                               // RH4 as input (ADC), others as output (7-segment digit select)
    PORTA = 0x00;                               // Clear PORTA output
    PORTH = 0x00;                               // Clear PORTH output
    TRISB = 0x10;                               // RB4 as input (button), others as output
    LATB = 0x00;                                // Clear PORTB output latches
    PORTB = 0x00;                               // Clear PORTB output
}                                                                          
/**
 * Initialize Timer0 for 5ms periodic interrupts
 * Used for timing control of 100ms message transmission and parking duration tracking
 */
void timer_init(){
    T0CON = 0x08;                               // 16-bit mode, internal clock, no prescaler, timer enabled
    TMR0H = 0x3C;                               // High byte of timer reload value for 5ms period
    TMR0L = 0xAF;                               // Low byte of timer reload value for 5ms period
    T0CONbits.TMR0ON = 1;                       // Enable Timer0
}
/**
 * Timer0 interrupt service routine
 * Called every 5ms to handle timing-critical tasks
 */
void timer_isr(){
    INTCONbits.TMR0IF = 0;                      // Clear Timer0 interrupt flag
    TMR0H = 0x3C;                               // Reload timer high byte for next 5ms period
    TMR0L = 0xAF;                               // Reload timer low byte for next 5ms period
    five_ms_counter++;                          // Increment 5ms counter (used for parking time calculation)
    time_to_adc++;                              // Increment ADC timing counter
    if (++counter_for_output >= 20) {           // Check if 100ms period elapsed (20 * 5ms = 100ms)
        counter_for_output = 0;                 // Reset counter
        output_due = 1;                         // Signal that message output is due
    }
    if(time_to_adc>=100){                       // Check if 500ms elapsed for ADC sampling (100 * 5ms = 500ms)
        time_to_adc=0;                          // Reset ADC timing counter
        PIR1bits.ADIF = 0;                      // Clear ADC interrupt flag
        ADCON0bits.GO_nDONE = 1;                // Start ADC conversion
    }
}
/**
 * Initializes the ADC (Analog-to-Digital Converter) module for reading potentiometer values.
 * Configures 10-bit ADC with interrupt-based conversion completion detection.
 * The ADC is used to select parking levels (A, B, C, D) based on potentiometer position.
 */
void init_adc(void) {
    ADCON0 = 0x31; // Enable ADC (ADON=1), select channel AN12 (CHS=1100), start conversion ready
    ADCON1 = 0x00; // Configure all pins as analog inputs, use VDD and VSS as references
    ADCON2 = 0xAA; // Right justified result, acquisition time = 8 TAD, conversion clock = FOSC/32
    ADRESH = 0x00; // Clear high byte of ADC result register
    ADRESL = 0x00; // Clear low byte of ADC result register
}
/**
 * ADC interrupt service routine - handles completion of analog-to-digital conversion.
 * Called when ADC conversion is complete, reads the converted value and maps it to parking levels.
 * The 10-bit ADC result is read from ADRESH register (high 8 bits) for level selection.
 */
void adc_isr() {
    adc_value = 0; // Clear previous ADC value
    adc_value = ADRESH; // Read high 8 bits of 10-bit ADC result (sufficient for level mapping)
    PIR1bits.ADIF = 0; // Clear ADC interrupt flag to acknowledge interrupt
}
/**
 * High priority interrupt service routine - central interrupt handler for all high priority interrupts.
 * Handles multiple interrupt sources: USART receive/transmit, timer overflow, ADC conversion,
 * and PORTB change interrupts. Each interrupt source is checked and appropriate handler called.
 */
void __interrupt(high_priority) highPriorityISR(void) {
    if (PIR1bits.RC1IF) receive_isr(); // Handle USART receive interrupt when data arrives
    if (PIR1bits.TX1IF) transmit_isr(); // Handle USART transmit interrupt when ready to send
    if (INTCONbits.TMR0IF) timer_isr(); // Handle Timer0 overflow interrupt for timing control
    if (PIR1bits.ADIF) adc_isr(); // Handle ADC conversion complete interrupt
    if (INTCONbits.RBIF) {              // Handle PORTB change interrupt for button press detection
        volatile uint8_t dummyRead = PORTB; // Read PORTB to clear mismatch condition
        INTCONbits.RBIF = 0; // Clear PORTB interrupt flag
        current_RB4_state = PORTBbits.RB4; // Read current state of RB4 button
        // Detect button release (rising edge: 0 -> 1 transition)
        if (current_RB4_state == 1 && previous_RB4_state == 0) {
            // Toggle display mode between money ('m') and empty spaces ('e')
            if (display_mode == 'e') {
                display_mode = 'm'; // Switch to money display mode
            } 
            else if(display_mode == 'm'){
                display_mode = 'e'; // Switch to empty spaces display mode
            }
        }
        previous_RB4_state = current_RB4_state; // Update previous state for next comparison
    }
}
/**
 * Displays the number of empty parking spaces for a specified level on 7-segment display.
 * Updates the current digit position and calculates empty spaces for the requested level.
 * Only rightmost 2 digits are used in this mode, leftmost digits show blank.
 */
void show_empty_space_number(char wanted_level){
    current_digit++; // Move to next digit position
    if(current_digit>=4){
        current_digit = 0; // Wrap around to first digit after reaching last digit
    }
    int empty_place_count = 0; // Counter for empty spaces in the level
    int start_index = (wanted_level-'A')*10; // Calculate starting slot index for the level
    for(int i = start_index; i<start_index+10 ; i++){   // Count empty spaces in the specified level (10 slots per level)
        if(slot_list[i].is_empty==1){
            empty_place_count++; // Increment if slot is empty
        }
    }
    int space_into_digits[4];       // Break down count into individual digits for display
    space_into_digits[0]=empty_place_count%10; // Ones digit
    space_into_digits[1]=(empty_place_count/10)%10; // Tens digit
    space_into_digits[2]=10; // Blank digit (index 10 = blank in segment_values array)
    space_into_digits[3]=10; // Blank digit
    display_digit(space_into_digits[current_digit]); // Display current digit
} 
/**
 * Maps ADC value to parking level and displays corresponding empty space count.
 * Converts 8-bit ADC reading to level selection (A, B, C, D) and shows empty spaces.
 * ADC ranges: 0-63=A, 64-127=B, 128-191=C, 192-255=D (simplified from 10-bit ranges).
 */
void value_to_level(uint8_t current_adc_value){      
    char level_from_adc;            // Selected level based on ADC value
    switch(current_adc_value){      // Selected level based on ADC value
        case 0:
            level_from_adc='A';
            break;
        case 1:
            level_from_adc='B';
            break;
        case 2:
            level_from_adc='C';
            break;
        case 3:
            level_from_adc='D';
            break;
    }
    show_empty_space_number(level_from_adc);
}
/**
 * Main calculation and command processing task - handles all incoming commands from simulator.
 * Processes GO, END, PARK (PRK), SUBSCRIPTION (SUB), and EXIT (EXT) commands.
 * Manages parking lot state, fee calculations, and response message generation.
 * This is the core logic function that maintains parking lot operations.
 */
void calc_task(){
    int fee=0;
    char plaka[4];
    char message_to_enqueue[12];        // Buffer for outgoing messages
    if(pkt_valid==0){
        return;                         // No valid packet to process
    }
    if (pkt_body[0]=='G' && pkt_body[1]=='O'){              // Handle GO command - start simulation
        timer_init();                                       // Initialize timer for periodic operations
        start_seven_segment();
        pkt_valid = 0;                                      // Mark packet as processed
        return;
    }
    else if (pkt_body[0]=='E' && pkt_body[1]=='N'){         // Handle END command - stop simulation
        INTCONbits.GIE = 0;                                 // Disable global interrupts
        end_flag = 1;                                       // Set flag to exit main loop
        pkt_valid = 0;                                      // Mark packet as processed
        return; 
    }
    else {
        char parked_level,parked_high,parked_low;           // Parking slot assignment variables
        plaka[0] = pkt_body[3];                             // Extract license plate from command (positions 3,4,5)
        plaka[1] = pkt_body[4];
        plaka[2] = pkt_body[5];
        plaka[3] = '\0';
        if (pkt_body[0]=='P' && pkt_body[1]=='R'){          // Handle PARK command (PRK) - car wants to park
            for(int i=0 ; i<40; i++){                       // Check if car has a reservation
                if ((slot_list[i].plate[0]==pkt_body[3]) && (slot_list[i].plate[1]==pkt_body[4])&& (slot_list[i].plate[2]==pkt_body[5])){
                    if(slot_list[i].is_reserved){           // Car found with reservation
                        if(slot_list[i].is_empty){
                            slot_list[i].is_empty=0;        // Mark slot as occupied
                            message_to_enqueue[0]='$';      // Prepare parking space assignment message
                            message_to_enqueue[1]='S';
                            message_to_enqueue[2]='P';
                            message_to_enqueue[3]='C';
                            message_to_enqueue[4]=slot_list[i].plate[0];
                            message_to_enqueue[5]=slot_list[i].plate[1];
                            message_to_enqueue[6]=slot_list[i].plate[2];
                            message_to_enqueue[7]=slot_list[i].level;
                            message_to_enqueue[8]=slot_list[i].high;
                            message_to_enqueue[9]=slot_list[i].low;
                            message_to_enqueue[10]='#';
                            message_to_enqueue[11]='\0';
                            message_enqueue(message_to_enqueue);        // Add message to queue
                            TOTAL_EMPTY_SPACE--;// Decrease available space count
                            TOTAL_EMPTY_RES_SPACE--;
                        }
                    }
                    goto end_of_calc_task;                      // Skip to end if reservation handled
                }
            }
            // No reservation found - handle as regular parking
            if ((plate_count == 0) && (TOTAL_EMPTY_SPACE-TOTAL_EMPTY_RES_SPACE) > 0){ 
                // Space available and no queue - park immediately
                TOTAL_EMPTY_SPACE--;  // Decrease available space
                for (int k = 0; k < 40; k++) {
                    if (slot_list[k].is_empty && !slot_list[k].is_reserved) {
                        for (int p = 0 ; p<3 ; p++){        // Assign plate to slot
                            slot_list[k].plate[p]=plaka[p];
                        }
                        slot_list[k].plate[3]='\0';
                        slot_list[k].start_time = five_ms_counter;      // Record parking start time
                        slot_list[k].is_empty = 0;
                        parked_level = slot_list[k].level;              // Store slot details for message
                        parked_high = slot_list[k].high;
                        parked_low = slot_list[k].low;
                        break;
                    }
                }
                // Prepare parking space assignment message
                message_to_enqueue[0]='$';          
                message_to_enqueue[1]='S';
                message_to_enqueue[2]='P';
                message_to_enqueue[3]='C';
                message_to_enqueue[4]=plaka[0];
                message_to_enqueue[5]=plaka[1];
                message_to_enqueue[6]=plaka[2];
                message_to_enqueue[7]=parked_level;
                message_to_enqueue[8]=parked_high;
                message_to_enqueue[9]=parked_low;
                message_to_enqueue[10]='#';
                message_to_enqueue[11]='\0';
                message_enqueue(message_to_enqueue);
            }
            else{
                // No space available - add to waiting queue
                plate_enqueue(plaka);
            }
            pkt_valid=0;
            return;
        }
        // Handle SUBSCRIPTION command (SUB) - car wants to reserve a space
        else if (pkt_body[0]=='S' && pkt_body[1]=='U'){
            // Find the requested parking slot
            for (int i = 0; i<40 ; i++){
                if ( (slot_list[i].level == pkt_body[6]) && (slot_list[i].high == pkt_body[7])&&(slot_list[i].low == pkt_body[8])){
                    // Check if slot is available for reservation
                    if (slot_list[i].is_empty&&!(slot_list[i].is_reserved)){
                        // Slot available - successful reservation
                        message_to_enqueue[0]='$';
                        message_to_enqueue[1]='R';
                        message_to_enqueue[2]='E';
                        message_to_enqueue[3]='S';
                        message_to_enqueue[4]=plaka[0];
                        message_to_enqueue[5]=plaka[1];
                        message_to_enqueue[6]=plaka[2];
                        message_to_enqueue[7]='5';
                        message_to_enqueue[8]='0';
                        message_to_enqueue[9]='#';
                        message_to_enqueue[10]='\0';
                        message_to_enqueue[11]='\0';
                        slot_list[i].is_empty=1;        // Keep as empty but reserved
                        slot_list[i].is_reserved = 1;   // Mark as reserved
                        for (int g = 0 ; g<3 ; g++){    // Assign plate to reserved slot
                            slot_list[i].plate[g]=plaka[g];
                        }
                        slot_list[i].plate[3]='\0';
                        TOTAL_MONEY_EARNED+=50;         // Add subscription fee
                        TOTAL_RES_SPACE ++;
                        TOTAL_EMPTY_RES_SPACE++;
                    }
                    else{   // Slot not available - failed reservation
                        message_to_enqueue[0]='$';
                        message_to_enqueue[1]='R';
                        message_to_enqueue[2]='E';
                        message_to_enqueue[3]='S';
                        message_to_enqueue[4]=plaka[0];
                        message_to_enqueue[5]=plaka[1];
                        message_to_enqueue[6]=plaka[2];
                        message_to_enqueue[7]='0';
                        message_to_enqueue[8]='0';
                        message_to_enqueue[9]='#';
                        message_to_enqueue[10]='\0';
                        message_to_enqueue[11]='\0';
                    }
                    break;
                }
            }
            message_enqueue(message_to_enqueue);
            pkt_valid=0;
            return;
        }
        else if (pkt_body[0]=='E' && pkt_body[1]=='X'){         // Handle EXIT command (EXT) - car wants to leave
            // Find the car in parking slots
            for(int j = 0; j<40 ; j++){
                if((slot_list[j].plate[0]==pkt_body[3]) && (slot_list[j].plate[1]==pkt_body[4])&& (slot_list[j].plate[2]==pkt_body[5])){ 
                    // Handle reserved car exit
                    if(slot_list[j].is_reserved && !slot_list[j].is_empty){
                        slot_list[j].is_empty=1;            // Mark slot as empty but keep reservation
                        message_to_enqueue[0]='$';          // Prepare fee message (0 for subscribed cars)
                        message_to_enqueue[1]='F';
                        message_to_enqueue[2]='E';
                        message_to_enqueue[3]='E';
                        message_to_enqueue[4]=plaka[0];
                        message_to_enqueue[5]=plaka[1];
                        message_to_enqueue[6]=plaka[2];
                        message_to_enqueue[7]='0';
                        message_to_enqueue[8]='0';
                        message_to_enqueue[9]='0';
                        message_to_enqueue[10]='#';
                        message_to_enqueue[11]='\0';
                        message_enqueue(message_to_enqueue);
                        TOTAL_EMPTY_SPACE++;             // Increase available space count
                        TOTAL_EMPTY_RES_SPACE++;
                        pkt_valid = 0;
                        return;    
                    }
                    else if (!slot_list[j].is_reserved && !slot_list[j].is_empty){          // Handle regular car exit with fee calculation
                        int integer_division = ( (int) (five_ms_counter-slot_list[j].start_time))/50;       // Calculate parking fee: (time_in_250ms_units) + 1
                        fee = integer_division + 1;
                        TOTAL_MONEY_EARNED += fee;
                        for(int d=0; d<4;d++){              // Clear slot information
                            slot_list[j].plate[d] = '\0';
                        }
                        slot_list[j].start_time=0;          // Reset start time
                        slot_list[j].is_empty=1;            // Mark as empty
                        char fee1 = '0' + (fee % 10);       // Convert fee to ASCII characters
                        char fee2 = '0' + ((fee / 10) % 10);
                        char fee3 = '0' + (((fee)/100) %10);
                        fee = 0;
                        message_to_enqueue[0]='$';
                        message_to_enqueue[1]='F';
                        message_to_enqueue[2]='E';
                        message_to_enqueue[3]='E';
                        message_to_enqueue[4]=plaka[0];
                        message_to_enqueue[5]=plaka[1];
                        message_to_enqueue[6]=plaka[2];
                        message_to_enqueue[7]=fee3;
                        message_to_enqueue[8]=fee2;
                        message_to_enqueue[9]=fee1;
                        message_to_enqueue[10]='#';
                        message_to_enqueue[11]='\0';
                        TOTAL_EMPTY_SPACE++;
                        message_enqueue(message_to_enqueue);
                        if(plate_count!=0){             // Check if there are cars waiting in queue
                            char plate1[4];             // Buffer for waiting car plate
                            plate_dequeue();             // Get next waiting car
                            for (int p = 0 ; p<3 ; p++){    // Copy dequeued plate
                                plate1[p]=dequeued_plate[p];
                            }
                            plate1[3]='\0';
                            for (int t = 0 ; t<3 ; t++){        // Assign waiting car to the vacated slot
                                slot_list[j].plate[t]=plate1[t];
                            }
                            slot_list[j].plate[3]='\0';
                            slot_list[j].start_time=five_ms_counter; // Set new start time
                            slot_list[j].is_empty=0; // Mark as occupied
                            slot_list[j].is_reserved=0; // Clear reservation flag
                            message_to_enqueue[0]='$';
                            message_to_enqueue[1]='S';
                            message_to_enqueue[2]='P';
                            message_to_enqueue[3]='C';
                            message_to_enqueue[4]=plate1[0];
                            message_to_enqueue[5]=plate1[1];
                            message_to_enqueue[6]=plate1[2];
                            message_to_enqueue[7]=slot_list[j].level;
                            message_to_enqueue[8]=slot_list[j].high;
                            message_to_enqueue[9]=slot_list[j].low;
                            message_to_enqueue[10]='#';
                            message_to_enqueue[11]='\0';
                            message_enqueue(message_to_enqueue);
                            TOTAL_EMPTY_SPACE --;
                        }
                        pkt_valid=0;
                        return;
                    }
                    else{
                        pkt_valid=0;
                        return;
                    }
                }
            }
        }
        pkt_valid = 0;
        return;    
    }
    end_of_calc_task:
    pkt_valid = 0;
    return;
}
/**
 * Main function - program entry point and main execution loop.
 * Initializes all system components and runs the main parking lot management loop.
 * Handles packet processing, calculations, periodic message transmission, and display updates.
 * Continues until END command is received from simulator.
 */
void main(void) { 
    init_queue();               // Initialize message and plate queues
    init_ports();               // Configure I/O ports
    init_serial();              // Initialize USART communication
    init_interrupts();          // Enable interrupt sources
    init_adc();                 // Initialize ADC for potentiometer reading
    start_system();             // Enable global interrupts
    init_slot();                // Initialize parking slot data structures
    
    while(1) {                  // Main execution loop - runs until end_flag is set
        packet_task();          // Process incoming serial packets
        calc_task();            // Process commands and manage parking lot state
        if (output_due) {       // Handle periodic message transmission (every 100ms)
            output_due = 0;     // Clear output flag
            message_dequeue();  // Get next message from queue
            if(message_var){
                message_var = 0;
                message_to_send[11]='\0';
                output_str(message_to_send);            // Add message to output buffer
                output_task();                          // Initiate transmission
            }
            else{                                       // No queued messages - send empty space status
                char tens, ones;
                char emp_msg[8];
                tens = ((TOTAL_EMPTY_SPACE/10)%10)+'0'; 
                ones = ((TOTAL_EMPTY_SPACE)%10)+'0';
                emp_msg[0]='$';
                emp_msg[1]='E';
                emp_msg[2]='M';
                emp_msg[3]='P';
                emp_msg[4]=tens;
                emp_msg[5]=ones;
                emp_msg[6]='#';
                emp_msg[7]='\0';
                output_str(emp_msg);
                output_task();
            }
        }
        // Update 7-segment display based on current mode
        if(display_mode=='m'){
            show_total_money(); // Display total money earned
        }
        if(display_mode == 'e' ){
            value_to_level(adc_value); // Display empty spaces for selected level
        }
        
        // Check for simulation end condition
        if(end_flag){
            break; // Exit main loop when END command received
        }
    }
    return; // Program termination
}
