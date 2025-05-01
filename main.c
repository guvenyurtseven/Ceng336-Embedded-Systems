// ============================ //
// Do not edit this part!!!!    //
// ============================ //
// 0x300001 - CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator,
                                // PLL enabled (Clock Frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit
                                // (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit
                                // (Oscillator Switchover mode disabled)
// 0x300002 - CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out
                                // Reset disabled in hardware and software)
// 0x300003 - CONFIG1H
#pragma config WDT = OFF        // Watchdog Timer Enable bit
                                // (WDT disabled (control is placed on the SWDTEN bit))
// 0x300004 - CONFIG3L
// 0x300005 - CONFIG3H
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit
                                // (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled;
                                // RE3 input pin disabled)
// 0x300006 - CONFIG4L
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply
                                // ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit
                                // (Instruction set extension and Indexed
                                // Addressing mode disabled (Legacy mode))
#pragma config DEBUG = OFF      // Disable In-Circuit Debugger

// Timer Related Definitions
#define KHZ 1000UL
#define MHZ (KHZ * KHZ)
#define _XTAL_FREQ (40UL * MHZ)
// ============================ //
//             End              //
// ============================ //
#include <xc.h>


// ============================ //
//        DEFINITIONS           //
// ============================ //
#define FREQ 40000000   
// You can write struct definitions here...
typedef unsigned char uint8_t;

// ======================================================//
//                     GLOBALS  		         //
//						         //
//    const unsigned char Güven Yurtseven, 2449080       //
//    const unsigned char Ersin Sert, 2448819            //
//    const unsigned char Mehmet Özgür Göksu, 2310043    //
//						         //
// ======================================================//



// You can write globals definitions here...
uint8_t counter_for_blink = 0;
uint8_t counter_for_gravity = 0;
uint8_t counter_for_score_degrader = 0;
volatile int score = 0;
volatile int round_score = 100;
uint8_t bottom_index = 0x07;
uint8_t hippo_size = 1;
uint8_t blink_flag = 0;
uint8_t base_value = 1;
uint8_t pattern = 0x80;
uint8_t latd_password = 0x00;
uint8_t digit0 = 0;
uint8_t digit1 = 0;
uint8_t digit2 = 0;
uint8_t digit3 = 0;
uint8_t seven = 0x07;
uint8_t one = 0x01;
uint8_t zero = 0x00;
uint8_t shift_amount = 0;
volatile int i = 0;
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
    0x6F   // 9
};
// ============================ //
//          FUNCTIONS           //
// ============================ //

// You can write function definitions here...
void init()
{
    T0CON = 0x00;
    INTCON = 0x00;
    
    LATB = 0x00; PORTB = 0x00; TRISB = 0x01;
    LATD = 0x00; PORTD = 0x00; TRISD = 0x00;
    LATH = 0x00; PORTH = 0x00; TRISH = 0x00;
    LATJ = 0x00; PORTJ = 0x00; TRISJ = 0x00;
    PORTD = 0x81;
    blink_flag = one;
}
void interrupt_init()
{
    INTCON = 0xF0;
    INTCON2 = 0xC1;
    RCON = 0x80;
    T0CON = 0x88;
}
void display_digit(uint8_t digit, uint8_t position) 
{
    switch (position) {
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
    LATJ = segment_values[digit];
    return;
}

void reset_period()
{
    INTCON = 0x00;
    digit3 = (score / 1000) % 10;
    digit2 = (score / 100) % 10;
    digit1 = (score / 10) % 10;
    
    PORTD = 0xFF;
    for(i = 0; i<100 ; i++)
    {
        if( (i == 20) || (i == 60))
        {
            PORTD = 0x00;
        }
        if( (i == 40) || (i == 80))
        {
            PORTD = 0xFF;
        }
        
        display_digit(digit3, 3);
        __delay_ms(5); 
        display_digit(digit2, 2);
        __delay_ms(5);   
        display_digit(digit1, 1);
        __delay_ms(5);   
        display_digit(0, 0);
        __delay_ms(5);
    }
    PORTD = 0x01;  
    INTCON = 0xF0;
    return;
}

void hard_reset()
{
    score = score + round_score;
    
    round_score = 100;
    
    reset_period();
    
    blink_flag = one;
    
    pattern = 0x80;
    
    latd_password = pattern + blink_flag;
    
    PORTD = latd_password;
    
    hippo_size = one;
    
    bottom_index = seven;
    
    counter_for_score_degrader = 0;
    counter_for_gravity = 0;
    counter_for_blink = 0;
    
    return;
}

void soft_reset()
{
    score = score + round_score;
    
    round_score = 100;
    
    reset_period();
        
    blink_flag = one;
    
    shift_amount = seven - bottom_index;
    
    pattern = ( pattern << shift_amount ) + (base_value << shift_amount);
    
    base_value = 1;
    
    latd_password = pattern + blink_flag;
    
    PORTD = latd_password;
        
    bottom_index = seven;
    
    hippo_size++;
    
    counter_for_score_degrader = 0;
    counter_for_gravity = 0;
    counter_for_blink = 0;
    
    return;
}



// ============================ //
//   INTERRUPT SERVICE ROUTINE  //
// ============================ //
__interrupt(high_priority)
void HandleInterrupt1()
{
   if (INT0IF) 
   {
       INT0IF = 0;
       
       if(bottom_index == hippo_size) // reset ?art? kontrolü
       {
           if(hippo_size < 5)   // reset olmas? gerekiyorsa hard reset mi soft reset mi
           {
               soft_reset();
               return;
           }
           
           else
           {
               hard_reset();
               return;
           }
           
       }
       else
       {
           pattern = pattern >> one;
           
           latd_password = pattern + blink_flag;
           
           PORTD = latd_password;
           
           bottom_index--;
           
           return;
       }
   }
   return;
}

__interrupt(low_priority)
void HandleInterrupt2()
{
    if (TMR0IF)
    {  
        TMR0IF = 0;
        TMR0H = 0x3C;  
        TMR0L = 0xAF;
        counter_for_blink++;
        counter_for_gravity++;
        counter_for_score_degrader++;
        
        if (counter_for_blink>= 100)
        {
            PORTDbits.RD0 = !PORTDbits.RD0;
            if(blink_flag)
            {
                blink_flag = zero;
            }
            else
            {
                blink_flag = one;
            }
            counter_for_blink = zero;
        }
        
        
        if (counter_for_gravity>=70)
        {
            if(bottom_index < seven)
            {
                bottom_index++;
                pattern = pattern << one;
                latd_password = pattern + blink_flag;
                PORTD = latd_password;
            }
            
            counter_for_gravity = 0;
        }
        
        if (counter_for_score_degrader>=200)
        {
            if(round_score>10)
            {
                round_score=round_score-10;
            }
            else
            {
                round_score = 0;
            }
            counter_for_score_degrader = 0;
        }
        return;
    }
    
}

// ============================ //
//            MAIN              //
// ============================ //
void main()
{
    init();

    
    for(i=0;i<50;i++)
    {
        display_digit(digit3, 0);
        __delay_ms(5);
        display_digit(digit2, 0);
        __delay_ms(5);
        display_digit(digit1, 0);
        __delay_ms(5);
        display_digit(digit0, 0);
        __delay_ms(5);
    }
    
    interrupt_init();
    
    while(1)
    {
        digit3 = (score / 1000) % 10;
        digit2 = (score / 100) % 10;
        digit1 = (score / 10) % 10;
        display_digit(digit3, 3);
        __delay_ms(5); 
        display_digit(digit2, 2);
        __delay_ms(5);   
        display_digit(digit1, 1);
        __delay_ms(5);   
        display_digit(0, 0);
        __delay_ms(5);   
        
    }
}