; StartingCodeWithLog.asm
; Created by Kevin Johnson
; (no copyright applied; edit freely, no attribution necessary)
; This program includes:
; - Wireless position logging using timer interrupt
; - Subroutines for
; -- 16x16 signed multiplication
; -- 16/16 signed division
; -- Arctangent (in appropriate robot units)
; -- Distance (L2 norm) approximation
; - Example of using tables (ILOAD, ISTORE)
; - Additional misc. examples


;***************************************************************
;* Jump Table
;***************************************************************
; When an interrupt occurs, execution is redirected to one of
; these addresses (depending on the interrupt source), which
; needs to either contain RETI (return from interrupt) if not
; used, or a JUMP instruction to the desired interrupt service
; routine (ISR).  The first location is the reset vector, and
; should be a JUMP instruction to the beginning of your normal
; code.
ORG        &H000       ; Jump table is located in mem 0-4
	JUMP   Init        ; Reset vector
	RETI               ; Sonar interrupt (unused)
	JUMP   CTimer_ISR  ; Timer interrupt
	RETI               ; UART interrupt (unused)
	RETI               ; Motor stall interrupt (unused)

;***************************************************************
;* Initialization
;***************************************************************
Init:
	; Always a good idea to make sure the robot
	; stops in the event of a reset.
	LOAD   Zero
	OUT    LVELCMD     ; Stop motors
	OUT    RVELCMD
	OUT    SONAREN     ; Disable sonar (optional)
	OUT    BEEP        ; Stop any beeping

	CALL   SetupI2C    ; Configure the I2C to read the battery voltage
	CALL   BattCheck   ; Get battery voltage (and end if too low).
	OUT    LCD         ; Display batt voltage on LCD

WaitForSafety:
	; Wait for safety switch to be toggled
	IN     XIO         ; XIO contains SAFETY signal
	AND    Mask4       ; SAFETY signal is bit 4
	JPOS   WaitForUser ; If ready, jump to wait for PB3
	IN     TIMER       ; We'll use the timer value to
	AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
	SHIFT  8           ; Shift over to LED17
	OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
	JUMP   WaitForSafety

WaitForUser:
	; Wait for user to press PB3
	IN     TIMER       ; We'll blink the LEDs above PB3
	AND    Mask1
	SHIFT  5           ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue
	JUMP   Main

;***************************************************************
;* Main code
;***************************************************************
Main: ; "Real" program starts here.
	; You will probably want to reset the position at the start your project code:
	OUT    	RESETPOS    ; reset odometer in case wheels moved after programming
	CALL   	UARTClear   ; empty the UART receive FIFO of any old data
	LOAD	-4,  1


move:
	LOAD FFast
	OUT RVELCMD
	OUT LVELCMD
	IN RPOS
	SUB TwoFeet
	SUB TwoFeet
	JNEG move


waitm:
	LOAD Zero
	ADD FFast
	OUT LVELCMD
	SUB FFast
	SUB FFast
	OUT RVELCMD
	IN THETA
	ADDI -315
	JPOS waitm
	CALL DIE



;***************************************************************
;* Set Current Points
;* Saves your X and Y position that the pointer in inputs
;* is pointing to
;***************************************************************
CurrPos:
	LOAD InputArr
	STORE Pointer
	ILOAD Pointer
	STORE CurX
	CALL IncrementPtr
	ILOAD Pointer
	STORE CurY
	CALL IncrementPtr
	Return


;***************************************************************
;* Increment Pointer
;* Moves the pointer down one in the array
;***************************************************************
IncrementPtr:
	LOAD Pointer
	ADDI 1
	STORE Pointer
	Return


; This table is used in example 1.  Remember: DW puts these
; values in memory, and since SCOMP has unified memory, it
; doesn't much matter where these end up, as long as they don't
; get executed.
Table1:
DW 55
DW 72
DW 0
; Will use this as a pointer.  It's just a normal variable.
Ptr1: DW 0

Example1:
; Example 1: using tables with ILOAD and ISTORE.
; We'll add two numbers and put the result back in the table.
	; LOADI takes an immediate, and here we're giving it the address
	; of the table.  This will load AC with the address of the table.
	LOADI  Table1
	; Now we store that address in a variable
	STORE  Ptr1 ; pointer to the table
	; ILOAD (indirect load; not to be confused with LOADI) fetches the
	; contents of memory at the address contained in a variable.  Since
	; Ptr1 contains the address of the table, the following instruction
	; will load AC with the first value in the table
	ILOAD  Ptr1  ; get table value
	STORE  Temp  ; keep first table value for later
	LOAD   Ptr1
	ADDI   1     ; increment the pointer
	STORE  Ptr1  ; don't forget to store the new pointer value
	ILOAD  Ptr1  ; get the second table value
	ADD    Temp  ; add the first table value
	STORE  Temp  ; save sum for later
	LOAD   Ptr1
	ADDI   1     ; increment the pointer (now at third value)
	STORE  Ptr1
	LOAD   Temp  ; get the sum back in AC
	; Like ILOAD, ISTORE operates on the memory location contained
	; in a variable.
	ISTORE Ptr1  ; put the sum in memory at the third table entry

	; To prove that everything worked:
	LOADI  Table1 ; get the table address fresh
	ADDI   2     ; increment address to result entry
	STORE  Temp  ; different variable to show that 'Ptr1' is nothing special
	LOADI  0     ; clear the AC just to prove we're getting the table value
	ILOAD  Temp  ; get the table value (3rd entry)
	OUT    LCD   ; and display it for debugging purposes
	; 55+72 = 127, or 0x7F

Example2:
; Example 2: multiply and divide subroutines.
; Included in this file are subroutines for 16-bit
; signed multiply and divide.
; Very important: multiplying two 16-bit numbers gives a
; 32-bit result.  You need to pay attention to the size
; of your numbers.
; See the subroutines below for specific information.
	; Multiply:
	LOADI  1003     ; LOADI can load numbers up to 1023
	STORE  m16sA    ; this is one input to the mult subroutine
	LOADI  -1019
	STORE  m16sB    ; this is the other number to multiply
	CALL   Mult16s  ; call this to perform the multiplication
	LOAD   mres16sH ; high word of the 32-bit result
	OUT    SSEG1
	LOAD   mres16sL ; low word of the 32-bit result
	OUT    SSEG2
	; 1003*-1019 = -1022057, or 0xFFF0_6797
	; Note that just taking the low word would give you
	; the completely wrong result (0x6797 = 26519).

	; Divide:
	LOADI  1003
	SHIFT  3
	ADDI   334      ; 1003*8+334 = 8358
	STORE  d16sN    ; this is the numerator to the div subroutine
	LOADI  -29
	STORE  d16sD    ; this is the denominator
	CALL   Div16s   ; call this to perform the division
	LOAD   dres16sQ ; quotient of division
	OUT    LEDs
	LOAD   dres16sR ; remainder of division
	OUT    XLEDs
	; 8358/-29 = -288 R6 = 0b1111111011100000 R0b0110


	; wait here for a few seconds so you can see the results.
	; you can also reset, if you want to re-run examples 1 & 2
	LOADI  30       ; wait 3 seconds
	CALL   WaitAC

Example3:
; Example 3: Angle and distance calculations
; Once SCOMP enters this piece of code, you should roll it around
; the floor.  The angle from (0,0) to the current position will
; be displayed on sseg1, and the distance from (0,0) to the current
; position will be displayed on sseg2.
; Also, since position logging is enabled at the beginning
; (by calling StartLog), the robot will be sending its current
; X/Y coordinate to the server every half a second.
; Finally, the first time the robot gets >2ft from (0,0), this
; code will call the IndicatePos routine, which you will use to
; tell the server when you reach each of your destinations.
	LOADI  0
	STORE  Tripped  ; used to indicate conditions in following code
	; clear all displays
	OUT    XLEDS
	OUT    LEDS
	OUT    LCD
	OUT    SSEG1
	OUT    SSEG2
	CALL   StartLog    ; enable the interrupt-based position logging
E3Loop:
	IN     XPOS
	STORE  AtanX      ; input to atan subroutine
	STORE  L2X        ; input to distance estimation subroutine
	IN     YPOS
	STORE  AtanY      ; input to atan subroutine
	STORE  L2Y        ; input to distance estimation subroutine
	CALL   Atan2      ; find the angle
	OUT    SSEG1
	CALL   L2Estimate ; estimate the distance
	OUT    SSEG2

	SUB    TwoFeet
	JPOS   Over2Ft    ; if over 2ft, trip the indicator
	JUMP   E3Loop     ; repeat forever
Over2Ft:
	LOAD   Tripped
	JPOS   E3Loop     ; if already indicated, don't do it again
	LOAD   TripCount  ; this example passes an incrementing count
	                  ; to the IndicateDest subroutine.  Your code
					  ; should pass the current destination number.
	CALL   IndicateDest
	LOADI  1
	STORE  Tripped    ; remember that already indicated this round
	LOAD   TripCount
	ADDI   1
	STORE  TripCount  ; increment the counter
	LOAD   NegOne
	OUT    LEDS       ; display for debug
	JUMP   E3Loop     ; repeat forever
Tripped: DW 0
TripCount: DW 0
; note on TripCount: when the DE2 first gets programmed (from Quartus),
; TripCount will be 0 in memory.  The above code increments it, but
; never sets it back to 0.  That means that resetting SCOMP with
; KEY0 will maintain TripCount's value.  So TripCount will continue
; to increment, even though you reset SCOMP with KEY0.



Die:
; Sometimes it's useful to permanently stop execution.
; This will also catch the execution if it accidentally
; falls through from above.
	LOAD   Zero        ; Stop everything.
	OUT    LVELCMD
	OUT    RVELCMD
	OUT    SONAREN
	LOAD   DEAD        ; An indication that we are dead
	OUT    SSEG2
	CALL   StopLog     ; Disable position logging
Forever:
	JUMP   Forever     ; Do this forever.
DEAD:      DW &HDEAD   ; Example of a "local variable"


;***************************************************************
;* Subroutines
;***************************************************************

; Subroutine to wait (block) for 1 second
Wait1:
	OUT    TIMER
Wloop:
	IN     TIMER
	ADDI   -10         ; 1 second in 10Hz.
	JNEG   Wloop
	RETURN

; Subroutine to wait the number of timer counts currently in AC
WaitAC:
	STORE  WaitTime
	OUT    Timer
WACLoop:
	IN     Timer
	SUB    WaitTime
	JNEG   WACLoop
	RETURN
	WaitTime: DW 0     ; "local" variable.

; This subroutine will get the battery voltage,
; and stop program execution if it is too low.
; SetupI2C must be executed prior to this.
BattCheck:
	CALL   GetBattLvl
	JZERO  BattCheck   ; A/D hasn't had time to initialize
	SUB    MinBatt
	JNEG   DeadBatt
	ADD    MinBatt     ; get original value back
	RETURN
; If the battery is too low, we want to make
; sure that the user realizes it...
DeadBatt:
	LOAD   Four
	OUT    BEEP        ; start beep sound
	CALL   GetBattLvl  ; get the battery level
	OUT    SSEG1       ; display it everywhere
	OUT    SSEG2
	OUT    LCD
	LOAD   Zero
	ADDI   -1          ; 0xFFFF
	OUT    LEDS        ; all LEDs on
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	Load   Zero
	OUT    BEEP        ; stop beeping
	LOAD   Zero
	OUT    LEDS        ; LEDs off
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	JUMP   DeadBatt    ; repeat forever

; Subroutine to read the A/D (battery voltage)
; Assumes that SetupI2C has been run
GetBattLvl:
	LOAD   I2CRCmd     ; 0x0190 (write 0B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	IN     I2C_DATA    ; get the returned data
	RETURN

; Subroutine to configure the I2C for reading batt voltage
; Only needs to be done once after each reset.
SetupI2C:
	CALL   BlockI2C    ; wait for idle
	LOAD   I2CWCmd     ; 0x1190 (write 1B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD register
	LOAD   Zero        ; 0x0000 (A/D port 0, no increment)
	OUT    I2C_DATA    ; to I2C_DATA register
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	RETURN

; Subroutine to block until I2C device is idle
BlockI2C:
	LOAD   Zero
	STORE  Temp        ; Used to check for timeout
BI2CL:
	LOAD   Temp
	ADDI   1           ; this will result in ~0.1s timeout
	STORE  Temp
	JZERO  I2CError    ; Timeout occurred; error
	IN     I2C_RDY     ; Read busy signal
	JPOS   BI2CL       ; If not 0, try again
	RETURN             ; Else return
I2CError:
	LOAD   Zero
	ADDI   &H12C       ; "I2C"
	OUT    SSEG1
	OUT    SSEG2       ; display error message
	JUMP   I2CError

; Subroutines to send AC value through the UART,
; Calling UARTSend2 will send both bytes of AC
; formatted for default base station code:
; [ AC(15..8) | AC(7..0)]
; Calling UARTSend1 will only send the low byte.
; Note that special characters such as \lf are
; escaped with the value 0x1B, thus the literal
; value 0x1B must be sent as 0x1B1B, should it occur.
UARTSend2:
	STORE  UARTTemp
	SHIFT  -8
	ADDI   -27   ; escape character
	JZERO  UEsc1
	ADDI   27
	OUT    UART_DAT
	JUMP   USend2
UEsc1:
	ADDI   27
	OUT    UART_DAT
	OUT    UART_DAT
USend2:
	LOAD   UARTTemp
UARTSend1:
	AND    LowByte
	ADDI   -27   ; escape character
	JZERO  UEsc2
	ADDI   27
	OUT    UART_DAT
	RETURN
UEsc2:
	ADDI   27
	OUT    UART_DAT
	OUT    UART_DAT
	RETURN
	UARTTemp: DW 0

; Subroutine to send a newline to the computer log
UARTNL:
	LOAD   NL
	OUT    UART_DAT
	SHIFT  -8
	OUT    UART_DAT
	RETURN
	NL: DW &H0A1B

; Subroutine to send a space to the computer log
UARTNBSP:
	LOAD   NBSP
	OUT    UART_DAT
	SHIFT  -8
	OUT    UART_DAT
	RETURN
	NBSP: DW &H201B

; Subroutine to clear the internal UART receive FIFO.
UARTClear:
	IN     UART_DAT
	JNEG   UARTClear
	RETURN

; Subroutine to tell the server that this position is one
; of the destinations.  Use AC=0 for generic indication,
; or AC=#1-12 for specific indication
IndicateDest:
	; AC contains which destination this is
	AND    LowNibl    ; keep only #s 0-15
	STORE  IDNumber
	LOADI  1
	STORE  IDFlag     ; set flag for indication
	RETURN
	IDNumber: DW 0
	IDFlag: DW 0


; Timer interrupt, used to send position data to the server
CTimer_ISR:
	CALL   UARTNL ; newline
	IN     XPOS
	CALL   UARTSend2
	IN     YPOS
	CALL   UARTSend2
	LOAD   IDFlag ; check if user has request a destination indication
	JPOS   CTIndicateDest ; if yes, do it; otherwise...
	RETI   ; return from interrupt
CTIndicateDest:
	LOAD   IDNumber
	CALL   UARTSend1 ; send the indicated destination
	LOADI  0
	STORE  IDFlag
	RETI

; Configure the interrupt timer and enable interrupts
StartLog:
	; See supporting information on the powersof2 site for how
	; SCOMP's communication system works.
	CALL   UARTNL      ; send a newline to separate data
	LOADI  0
	STORE  IDFlag      ; clear any pending flag
	LOADI  50
	OUT    CTIMER      ; configure timer for 0.01*50=0.5s interrupts
	CLI    &B0010      ; clear any pending interrupt from timer
	SEI    &B0010      ; enable interrupt from timer (source 1)
	RETURN

; Disable the interrupt timer and interrupts
StopLog:
	CLI    &B0010      ; disable interrupt source 1 (timer)
	LOADI  0
	OUT    CTIMER      ; reset configurable timer
	RETURN

;******************************************************************************;
; Atan2: 4-quadrant arctangent calculation                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Original code by Team AKKA, Spring 2015.                                     ;
; Based on methods by Richard Lyons                                            ;
; Code updated by Kevin Johnson to use software mult and div                   ;
; No license or copyright applied.                                             ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; To use: store dX and dY in global variables AtanX and AtanY.                 ;
; Call Atan2                                                                   ;
; Result (angle [0,359]) is returned in AC                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Requires additional subroutines:                                             ;
; - Mult16s: 16x16->32bit signed multiplication                                ;
; - Div16s: 16/16->16R16 signed division                                       ;
; - Abs: Absolute value                                                        ;
; Requires additional constants:                                               ;
; - One:     DW 1                                                              ;
; - NegOne:  DW -1                                                              ;
; - LowByte: DW &HFF                                                           ;
;******************************************************************************;
Atan2:
	LOAD   AtanY
	CALL   Abs          ; abs(y)
	STORE  AtanT
	LOAD   AtanX        ; abs(x)
	CALL   Abs
	SUB    AtanT        ; abs(x) - abs(y)
	JNEG   A2_sw        ; if abs(y) > abs(x), switch arguments.
	LOAD   AtanX        ; Octants 1, 4, 5, 8
	JNEG   A2_R3
	CALL   A2_calc      ; Octants 1, 8
	JNEG   A2_R1n
	RETURN              ; Return raw value if in octant 1
A2_R1n: ; region 1 negative
	ADDI   360          ; Add 360 if we are in octant 8
	RETURN
A2_R3: ; region 3
	CALL   A2_calc      ; Octants 4, 5
	ADDI   180          ; theta' = theta + 180
	RETURN
A2_sw: ; switch arguments; octants 2, 3, 6, 7
	LOAD   AtanY        ; Swap input arguments
	STORE  AtanT
	LOAD   AtanX
	STORE  AtanY
	LOAD   AtanT
	STORE  AtanX
	JPOS   A2_R2        ; If Y positive, octants 2,3
	CALL   A2_calc      ; else octants 6, 7
	XOR    NegOne
	ADDI   1            ; negate the angle
	ADDI   270          ; theta' = 270 - theta
	RETURN
A2_R2: ; region 2
	CALL   A2_calc      ; Octants 2, 3
	XOR    NegOne
	ADDI   1            ; negate the angle
	ADDI   90           ; theta' = 90 - theta
	RETURN
A2_calc:
	; calculates R/(1 + 0.28125*R^2)
	LOAD   AtanY
	STORE  d16sN        ; Y in numerator
	LOAD   AtanX
	STORE  d16sD        ; X in denominator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  AtanRatio
	STORE  m16sA
	STORE  m16sB
	CALL   A2_mult      ; X^2
	STORE  m16sA
	LOAD   A2c
	STORE  m16sB
	CALL   A2_mult
	ADDI   256          ; 256/256+0.28125X^2
	STORE  d16sD
	LOAD   AtanRatio
	STORE  d16sN        ; Ratio in numerator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  m16sA        ; <= result in radians
	LOAD   A2cd         ; degree conversion factor
	STORE  m16sB
	CALL   A2_mult      ; convert to degrees
	STORE  AtanT
	SHIFT  -7           ; check 7th bit
	AND    One
	JZERO  A2_rdwn      ; round down
	LOAD   AtanT
	SHIFT  -8
	ADDI   1            ; round up
	RETURN
A2_rdwn:
	LOAD   AtanT
	SHIFT  -8           ; round down
	RETURN
A2_mult: ; multiply, and return bits 23..8 of result
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8            ; move high word of result up 8 bits
	STORE  mres16sH
	LOAD   mres16sL
	SHIFT  -8           ; move low word of result down 8 bits
	AND    LowByte
	OR     mres16sH     ; combine high and low words of result
	RETURN
A2_div: ; 16-bit division scaled by 256, minimizing error
	LOADI  9            ; loop 8 times (256 = 2^8)
	STORE  AtanT
A2_DL:
	LOAD   AtanT
	ADDI   -1
	JPOS   A2_DN        ; not done; continue shifting
	CALL   Div16s       ; do the standard division
	RETURN
A2_DN:
	STORE  AtanT
	LOAD   d16sN        ; start by trying to scale the numerator
	SHIFT  1
	XOR    d16sN        ; if the sign changed,
	JNEG   A2_DD        ; switch to scaling the denominator
	XOR    d16sN        ; get back shifted version
	STORE  d16sN
	JUMP   A2_DL
A2_DD:
	LOAD   d16sD
	SHIFT  -1           ; have to scale denominator
	STORE  d16sD
	JUMP   A2_DL
AtanX:      DW 0
AtanY:      DW 0
AtanRatio:  DW 0        ; =y/x
AtanT:      DW 0        ; temporary value
A2c:        DW 72       ; 72/256=0.28125, with 8 fractional bits
A2cd:       DW 14668    ; = 180/pi with 8 fractional bits

;*******************************************************************************
; Mult16s:  16x16 -> 32-bit signed multiplication
; Based on Booth's algorithm.
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: does not work with factor B = -32768 (most-negative number).
; To use:
; - Store factors in m16sA and m16sB.
; - Call Mult16s
; - Result is stored in mres16sH and mres16sL (high and low words).
;*******************************************************************************
Mult16s:
	LOADI  0
	STORE  m16sc        ; clear carry
	STORE  mres16sH     ; clear result
	LOADI  16           ; load 16 to counter
Mult16s_loop:
	STORE  mcnt16s
	LOAD   m16sc        ; check the carry (from previous iteration)
	JZERO  Mult16s_noc  ; if no carry, move on
	LOAD   mres16sH     ; if a carry,
	ADD    m16sA        ;  add multiplicand to result H
	STORE  mres16sH
Mult16s_noc: ; no carry
	LOAD   m16sB
	AND    One          ; check bit 0 of multiplier
	STORE  m16sc        ; save as next carry
	JZERO  Mult16s_sh   ; if no carry, move on to shift
	LOAD   mres16sH     ; if bit 0 set,
	SUB    m16sA        ;  subtract multiplicand from result H
	STORE  mres16sH
Mult16s_sh:
	LOAD   m16sB
	SHIFT  -1           ; shift result L >>1
	AND    c7FFF        ; clear msb
	STORE  m16sB
	LOAD   mres16sH     ; load result H
	SHIFT  15           ; move lsb to msb
	OR     m16sB
	STORE  m16sB        ; result L now includes carry out from H
	LOAD   mres16sH
	SHIFT  -1
	STORE  mres16sH     ; shift result H >>1
	LOAD   mcnt16s
	ADDI   -1           ; check counter
	JPOS   Mult16s_loop ; need to iterate 16 times
	LOAD   m16sB
	STORE  mres16sL     ; multiplier and result L shared a word
	RETURN              ; Done
c7FFF: DW &H7FFF
m16sA: DW 0 ; multiplicand
m16sB: DW 0 ; multipler
m16sc: DW 0 ; carry
mcnt16s: DW 0 ; counter
mres16sL: DW 0 ; result low
mres16sH: DW 0 ; result high

;*******************************************************************************
; Div16s:  16/16 -> 16 R16 signed division
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: results undefined if denominator = 0.
; To use:
; - Store numerator in d16sN and denominator in d16sD.
; - Call Div16s
; - Result is stored in dres16sQ and dres16sR (quotient and remainder).
; Requires Abs subroutine
;*******************************************************************************
Div16s:
	LOADI  0
	STORE  dres16sR     ; clear remainder result
	STORE  d16sC1       ; clear carry
	LOAD   d16sN
	XOR    d16sD
	STORE  d16sS        ; sign determination = N XOR D
	LOADI  17
	STORE  d16sT        ; preload counter with 17 (16+1)
	LOAD   d16sD
	CALL   Abs          ; take absolute value of denominator
	STORE  d16sD
	LOAD   d16sN
	CALL   Abs          ; take absolute value of numerator
	STORE  d16sN
Div16s_loop:
	LOAD   d16sN
	SHIFT  -15          ; get msb
	AND    One          ; only msb (because shift is arithmetic)
	STORE  d16sC2       ; store as carry
	LOAD   d16sN
	SHIFT  1            ; shift <<1
	OR     d16sC1       ; with carry
	STORE  d16sN
	LOAD   d16sT
	ADDI   -1           ; decrement counter
	JZERO  Div16s_sign  ; if finished looping, finalize result
	STORE  d16sT
	LOAD   dres16sR
	SHIFT  1            ; shift remainder
	OR     d16sC2       ; with carry from other shift
	SUB    d16sD        ; subtract denominator from remainder
	JNEG   Div16s_add   ; if negative, need to add it back
	STORE  dres16sR
	LOADI  1
	STORE  d16sC1       ; set carry
	JUMP   Div16s_loop
Div16s_add:
	ADD    d16sD        ; add denominator back in
	STORE  dres16sR
	LOADI  0
	STORE  d16sC1       ; clear carry
	JUMP   Div16s_loop
Div16s_sign:
	LOAD   d16sN
	STORE  dres16sQ     ; numerator was used to hold quotient result
	LOAD   d16sS        ; check the sign indicator
	JNEG   Div16s_neg
	RETURN
Div16s_neg:
	LOAD   dres16sQ     ; need to negate the result
	XOR    NegOne
	ADDI   1
	STORE  dres16sQ
	RETURN
d16sN: DW 0 ; numerator
d16sD: DW 0 ; denominator
d16sS: DW 0 ; sign value
d16sT: DW 0 ; temp counter
d16sC1: DW 0 ; carry value
d16sC2: DW 0 ; carry value
dres16sQ: DW 0 ; quotient result
dres16sR: DW 0 ; remainder result

;*******************************************************************************
; Abs: 2's complement absolute value
; Returns abs(AC) in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Abs:
	JPOS   Abs_r
	XOR    NegOne       ; Flip all bits
	ADDI   1            ; Add one (i.e. negate number)
Abs_r:
	RETURN

;*******************************************************************************
; Mod180: modulo 180
; Returns AC%180 in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Mod180:
	JNEG   Mod180n      ; handle negatives
Mod180p:
	ADDI   -180
	JPOS   Mod180p      ; subtract 180 until negative
	ADDI   180          ; go back positive
	RETURN
Mod180n:
	ADDI   180          ; add 180 until positive
	JNEG   Mod180n
	ADDI   -180         ; go back negative
	RETURN

;*******************************************************************************
; L2Estimate:  Pythagorean distance estimation
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: this is *not* an exact function.  I think it's most wrong
; on the axes, and maybe at 45 degrees.
; To use:
; - Store X and Y offset in L2X and L2Y.
; - Call L2Estimate
; - Result is returned in AC.
; Result will be in same units as inputs.
; Requires Abs and Mult16s subroutines.
;*******************************************************************************
L2Estimate:
	; take abs() of each value, and find the largest one
	LOAD   L2X
	CALL   Abs
	STORE  L2T1
	LOAD   L2Y
	CALL   Abs
	SUB    L2T1
	JNEG   GDSwap    ; swap if needed to get largest value in X
	ADD    L2T1
CalcDist:
	; Calculation is max(X,Y)*0.961+min(X,Y)*0.406
	STORE  m16sa
	LOADI  246       ; max * 246
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8
	STORE  L2T2
	LOAD   mres16sL
	SHIFT  -8        ; / 256
	AND    LowByte
	OR     L2T2
	STORE  L2T3
	LOAD   L2T1
	STORE  m16sa
	LOADI  104       ; min * 104
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8
	STORE  L2T2
	LOAD   mres16sL
	SHIFT  -8        ; / 256
	AND    LowByte
	OR     L2T2
	ADD    L2T3     ; sum
	RETURN
GDSwap: ; swaps the incoming X and Y
	ADD    L2T1
	STORE  L2T2
	LOAD   L2T1
	STORE  L2T3
	LOAD   L2T2
	STORE  L2T1
	LOAD   L2T3
	JUMP   CalcDist
L2X:  DW 0
L2Y:  DW 0
L2T1: DW 0
L2T2: DW 0
L2T3: DW 0


;***************************************************************
;* Inputs
;***************************************************************
Pointer:	DW 0
CurX:		DW 0
CurY:		DW 0

InputArr:
X0:			DW 0
Y0:			DW 0
X1:			DW 0
Y1:			DW 0
X2:			DW 0
Y2:			DW 0
X3:			DW 0
Y3:			DW 0
X4:			DW 0
Y4:			DW 0
X5:			DW 0
Y5:			DW 0
X6:			DW 0
Y6:			DW 0
X7:			DW 0
Y7:			DW 0
X8:			DW 0
Y8:			DW 0
X9:			DW 0
Y9:			DW 0
X10:		DW 0
Y10:		DW 0
X11:		DW 0
Y11:		DW 0
X12:		DW 0
Y12:		DW 0
;***************************************************************
;* Variables
;***************************************************************
Temp:  DW 0 ; "Temp" is not a great name, but can be useful
Temp2: DW 0
Temp3: DW 0
CDX: DW 0      ; current desired X
CDY: DW 0      ; current desired Y
CDT: DW 0      ; current desired angle
CX:  DW 0      ; sampled X
CY:  DW 0      ; sampled Y
CT:  DW 0      ; sampled theta

;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
NegOne:   DW -1
Zero:     DW 0
One:      DW 1
Two:      DW 2
Three:    DW 3
Four:     DW 4
Five:     DW 5
Six:      DW 6
Seven:    DW 7
Eight:    DW 8
Nine:     DW 9
Ten:      DW 10

; Some bit masks.
; Masks of multiple bits can be constructed by ORing these
; 1-bit masks together.
Mask0:    DW &B00000001
Mask1:    DW &B00000010
Mask2:    DW &B00000100
Mask3:    DW &B00001000
Mask4:    DW &B00010000
Mask5:    DW &B00100000
Mask6:    DW &B01000000
Mask7:    DW &B10000000
LowByte:  DW &HFF      ; binary 00000000 1111111
LowNibl:  DW &HF       ; 0000 0000 0000 1111

; some useful movement values
OneMeter: DW 952       ; ~1m in 1.05mm units
HalfMeter: DW 476      ; ~0.5m in 1.05mm units
OneFoot:  DW 290       ; ~1ft in 1.05mm robot units
TwoFeet:  DW 581       ; ~2ft in 1.05mm units
Deg90:    DW 90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500

MinBatt:  DW 130       ; 13.0V - minimum safe battery voltage
I2CWCmd:  DW &H1190    ; write one i2c byte, read one byte, addr 0x90
I2CRCmd:  DW &H0190    ; write nothing, read one byte, addr 0x90

;***************************************************************
;* IO address space map
;***************************************************************
SWITCHES: EQU &H00  ; slide switches
LEDS:     EQU &H01  ; red LEDs
TIMER:    EQU &H02  ; timer, usually running at 10 Hz
XIO:      EQU &H03  ; pushbuttons and some misc. inputs
SSEG1:    EQU &H04  ; seven-segment display (4-digits only)
SSEG2:    EQU &H05  ; seven-segment display (4-digits only)
LCD:      EQU &H06  ; primitive 4-digit LCD display
XLEDS:    EQU &H07  ; Green LEDs (and Red LED16+17)
BEEP:     EQU &H0A  ; Control the beep
CTIMER:   EQU &H0C  ; Configurable timer for interrupts
LPOS:     EQU &H80  ; left wheel encoder position (read only)
LVEL:     EQU &H82  ; current left wheel velocity (read only)
LVELCMD:  EQU &H83  ; left wheel velocity command (write only)
RPOS:     EQU &H88  ; same values for right wheel...
RVEL:     EQU &H8A  ; ...
RVELCMD:  EQU &H8B  ; ...
I2C_CMD:  EQU &H90  ; I2C module's CMD register,
I2C_DATA: EQU &H91  ; ... DATA register,
I2C_RDY:  EQU &H92  ; ... and BUSY register
UART_DAT: EQU &H98  ; UART data
UART_RDY: EQU &H98  ; UART status
SONAR:    EQU &HA0  ; base address for more than 16 registers....
DIST0:    EQU &HA8  ; the eight sonar distance readings
DIST1:    EQU &HA9  ; ...
DIST2:    EQU &HAA  ; ...
DIST3:    EQU &HAB  ; ...
DIST4:    EQU &HAC  ; ...
DIST5:    EQU &HAD  ; ...
DIST6:    EQU &HAE  ; ...
DIST7:    EQU &HAF  ; ...
SONALARM: EQU &HB0  ; Write alarm distance; read alarm register
SONARINT: EQU &HB1  ; Write mask for sonar interrupts
SONAREN:  EQU &HB2  ; register to control which sonars are enabled
XPOS:     EQU &HC0  ; Current X-position (read only)
YPOS:     EQU &HC1  ; Y-position
THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
RESETPOS: EQU &HC3  ; write anything here to reset odometry to 0