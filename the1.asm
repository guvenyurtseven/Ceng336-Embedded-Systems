PROCESSOR 18F8722

#include <xc.inc>

; CONFIGURATION (DO NOT EDIT)
; CONFIG1H
CONFIG OSC = HSPLL      ; Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
CONFIG FCMEN = OFF      ; Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
CONFIG IESO = OFF       ; Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)
; CONFIG2L
CONFIG PWRT = OFF       ; Power-up Timer Enable bit (PWRT disabled)
CONFIG BOREN = OFF      ; Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
; CONFIG2H
CONFIG WDT = OFF        ; Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
; CONFIG3H
CONFIG LPT1OSC = OFF    ; Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
CONFIG MCLRE = ON       ; MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)
; CONFIG4L
CONFIG LVP = OFF        ; Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
CONFIG XINST = OFF      ; Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))
CONFIG DEBUG = OFF      ; Disable In-Circuit Debugger


GLOBAL var1
GLOBAL var2
GLOBAL var3
GLOBAL digit0
GLOBAL digit1
GLOBAL digit2
GLOBAL digit3
GLOBAL digit4
GLOBAL digit5
GLOBAL zero
GLOBAL ten

GLOBAL one
GLOBAL two
GLOBAL three
GLOBAL four
GLOBAL five

    
GLOBAL seq_index

GLOBAL transition_flags
GLOBAL porte_current
GLOBAL porte_previous
GLOBAL flags  
; Define space for the variables in RAM
PSECT udata_acs
var1:
    DS 1 ; Allocate 1 byte for var1
var2:
    DS 1 
var3:
    DS 1
digit0:
    DS 1
digit1:
    DS 1
digit2:
    DS 1
digit3:
    DS 1
digit4:
    DS 1
digit5:
    DS 1
ten:
    DS 1
zero:
    DS 1
porte_current:
    DS 1
porte_previous:
    DS 1
flags:
    DS 1
transition_flags:
    DS 1
seq_index:
    DS 1
   
one:
    DS 1
two:
    DS 1
three:
    DS 1
four:
    DS 1
five:
    DS 1
    
    
PSECT resetVec,class=CODE,reloc=2
resetVec:
    goto       main

PSECT CODE
main:
    clrf var1			
    clrf var2   		
    clrf var3   
    clrf digit0
    clrf digit1
    clrf digit2
    clrf digit3
    clrf digit4
    clrf digit5
    clrf ten
    clrf zero
    clrf seq_index
    
    clrf porte_current
    clrf porte_previous
    clrf flags
    clrf transition_flags
    
    
    movlw 2
    movwf digit0
    movlw 4
    movwf digit1
    movlw 4
    movwf digit2
    movlw 9
    movwf digit3
    movlw 0
    movwf digit4
    movlw 8
    movwf digit5
    movlw 10
    movwf ten
    
    movlw 0
    movwf seq_index
    
    movlw 1
    movwf one
    movlw 2
    movwf two
    movlw 3
    movwf three
    movlw 4
    movwf four
    movlw 5
    movwf five
    
        
    ; PORTC as output
    clrf TRISC
    setf PORTC
    setf LATC
    ; PORTD as output
    clrf TRISD
    setf PORTD
    setf LATD
    
    ; PORTE as input
    setf TRISE
    clrf PORTE
    clrf LATE

    
    call busy_wait
    
    clrf PORTD
    clrf LATD
    clrf PORTC
    clrf LATC
    


main_loop:
    call round_robin
    goto main_loop
    

round_robin:
    
    btfss flags, 6
    btg LATD, 0
    
    btfss flags, 6
    call display
    
    
    movlw   1
    movwf   var3
    
    outer_loop:
    
	movlw   130
	movwf   var2
	
	inner_loop:
    
	    loop_start:
    
    
		movff PORTE, porte_current
		
		movff porte_previous, WREG
		
		xorwf porte_current, 0
		
		movwf transition_flags
		
		movlw 0
		
		cpfseq transition_flags
		
		call find_transition
		
		
		movff porte_current, porte_previous
		
		
		incf    var1, f      ; increment dummy counter
		bnc     loop_start   ; repeat until carry occurs (after 256 increments)
		
	    decfsz  var2, f         ; decrement inner loop counter
	    bra     inner_loop      ; repeat inner loop if not zero
	    
	decfsz  var3, f         ; decrement outer loop counter
	bra     outer_loop      ; repeat outer loop if not zero
	
    return


	
busy_wait:
    movlw   6
    movwf   var3
    
    outer_loop_1:
	movlw   216
	nop
	movwf   var2
	nop
	nop
	nop
	nop
	nop
	nop 
	nop
	inner_loop_1:
	    loop_start_1:
		
		incf    var1, f      ; increment dummy counter
		bnc     loop_start_1   ; repeat until carry occurs (after 256 increments)
	    decfsz  var2, f         ; decrement inner loop counter
	    bra     inner_loop_1      ; repeat inner loop if not zero

	decfsz  var3, f         ; decrement outer loop counter
	bra     outer_loop_1      ; repeat outer loop if not zero

    return
    
    
display:
    movff seq_index, WREG
    index_5:
	cpfseq five
	goto index_4
	movff digit5, LATC
	movff zero, seq_index
	goto local_end
    index_4:
	cpfseq four
	goto index_3
	movff digit4, LATC
	movff five, seq_index
	goto local_end
    index_3:
	cpfseq three
	goto index_2
	movff digit3, LATC
	movff four, seq_index
	goto local_end
    index_2:
	cpfseq two  
	goto index_1
	movff digit2, LATC
	movff three, seq_index
	goto local_end
    index_1:
	cpfseq one
	goto index_0
	movff digit1, LATC
	movff two, seq_index
	goto local_end
    index_0:
	cpfseq zero
	nop
	movff digit0, LATC
	movff one, seq_index
	goto local_end
	
    local_end:
    return
    
   
find_transition:
    
    movff porte_previous, WREG
    
    cpfsgt porte_current
    
    call release_detected
    
    return
    
release_detected:
    
    btfsc transition_flags, 7
    call paus_detected
    
    btfsc transition_flags, 5
    call digit5_inc
    
    btfsc transition_flags, 4
    call digit4_inc
    
    btfsc transition_flags, 3
    call digit3_inc
    
    btfsc transition_flags, 2
    call digit2_inc
    
    btfsc transition_flags, 1
    call digit1_inc
    
    btfsc transition_flags, 0
    call digit0_inc
    
    return 
    
    
paus_detected:
    btg flags, 6
    return
    
digit5_inc:
    incf digit5
    
    movlw 10
    
    cpfslt digit5
    clrf digit5
    
    movlw 0
    return
    
    
digit4_inc:
    incf digit4
    
    movlw 10
    
    cpfslt digit4
    clrf digit4
    
    movlw 0
    return
    
    
digit3_inc:
    incf digit3
    
    movlw 10
    
    cpfslt digit3
    clrf digit3
    
    movlw 0
    return
    
    
digit2_inc:
    incf digit2
    
    movlw 10
    
    cpfslt digit2
    clrf digit2
    
    movlw 0
    return
    
    
digit1_inc:
    incf digit1
    
    movlw 10
    
    cpfslt digit1
    clrf digit1
    
    movlw 0
    return
    
    
digit0_inc:
    incf digit0
    
    movlw 10
    
    cpfslt digit0
    clrf digit0
    
    movlw 0
    return
    
    
end resetVec
