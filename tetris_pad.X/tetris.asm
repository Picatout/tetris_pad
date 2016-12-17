    include p12f1572.inc
    
    radix dec
    errorlevel 0, -302, -305
    
    __config _CONFIG1, _FOSC_ECH&_WDTE_OFF
    __config _CONFIG2, _PLLEN_OFF&_LVP_OFF 

#define SOUND_SUPPORT
    
FOSC equ 20000000 ; 20Mhz
TC equ 50 ; nanosecondes 
FCY equ FOSC/4    ; 5Mhz
TCY equ 200 ; nanoseconds


XSIZE equ 48  ; horizontal pixels
YSIZE equ 32  ; vertical pixels
BPL equ XSIZE/8 ; bytes per line 
BUFFER_SIZE equ  YSIZE*BPL  ; video buffer size
VIDEO_BUFFER equ 0x2000+(3*80-BUFFER_SIZE) ; buffer address
VERT_DLY equ 50
FIRST_VIDEO equ VERT_DLY+20  ; first video output line
LAST_VIDEO equ FIRST_VIDEO+4*YSIZE-1  ; last video output line
PIXEL_TXREG equ TXREG
TX_PIR equ PIR1
HORZ_DLY equ 10*5  ; there is 5 TCY/�sec
 
VIDEO_OUT equ RA1
VIDEO_LAT equ LATA
SYNC_OUT equ RA2 
SYNC_PWMDCL equ PWM3DCL ; duty cycle low byte
SYNC_PWMDCH equ PWM3DCH ; high byte    
SYNC_PWMPRL equ PWM3PRL ; pwm period low byte
SYNC_PWMPRH equ PWM3PRH ; high byte 
SYNC_PWMPH equ PWM3PH ; pwm phase
SYNC_PWMOF equ PWM3OF ; pwm offset
SYNC_PWMTMR equ PWM3TMR ; timer
SYNC_PWMCON equ PWM3CON ; control SFR
SYNC_PWMCLKCON equ PWM3CLKCON  ; clock control SFR
SYNC_PWMOFCON equ PWM3OFCON ; offset control SFR
SYNC_PWMLDCON equ PWM3LDCON ; load control SFR
SYNC_IE equ PWM3IE  ; interrupt enable bit
SYNC_IF equ PWM3IF  ; interrupt flag bit
SYNC_PIE equ PIE3 ; interrupt enable SFR
SYNC_PIR equ PIR3 ; interrupt flag SFR 
SYNC_PWMINTE equ PWM3INTE
SYNC_PWMINTF equ PWM3INTF
 
HORZ_PERIOD equ FOSC/15748-1 ; 15748 hertz
HORZ_PULSE equ 4700/TC ; 4.7�sec 
LAST_LINE equ 262

LED_PIN equ RA0
ADC_PIN equ RA4

#ifdef SOUND_SUPPORT 
AUDIO_PIN equ RA4
AUDIO_PWMDCL equ PWM2DCL
AUDIO_PWMDCH equ PWM2DCH
AUDIO_PWMPRL equ PWM2PRL
AUDIO_PWMPRH equ PWM2PRH
AUDIO_PWMTMR equ PWM2TMR
AUDIO_PWMCLKCON equ PWM2CLKCON
AUDIO_PWMLDCON equ PWM2LDCON
AUDIO_PWMCON equ PWM2CON
#endif
 
; game pad resource
; use ADC 
PAD_PIN equ ANSA4
PAD_CHS equ 3 
BTN_UP equ 0
BTN_DN equ 1
BTN_LT equ 2
BTN_RT equ 3
BTN_A equ 4
BTN_B equ 5
; threshold level for each button
; lower_btn_thr >= BTN_? < btn_thr
; exemple: BTN_UP if RT_THR<=Vadc<UP_THR  
VDD equ 1024
DN_THR equ 216 ;(VDD*(6/7+5/6)/2)>>2
LT_THR equ 209 ; (VDD*(5/6+4/5)/2)>>2
UP_THR equ 198 ;(VDD*(4/5+3/4)/2)>>2
RT_THR equ 181 ;(VDD*(3/4+2/3)/2)>>2
B_THR  equ 149 ; (VDD*(2/3+1/2)/2)>>2
A_THR  equ 64 ;(VDD/4)>>2
 
try_button macro btn, label 
    movlw btn
    subwf ADRESH,W
    skpnc
    bra label
    endm
    
;boolean flags 
F_VSYNC equ 0 ; vertical sync active
F_EVEN equ 1  ; even field
F_GTMR equ 2  ; game timer active
F_GSTOP equ 3 ; game stopped
F_SND equ 4 ; sound timer active 
F_COLL equ 5 ; collision flag
 
LFSR_TAPS equ 0xB4 ; xor mask
 
tcyDelay macro n ; delay in TCY cycles  5TCY per �sec.  
    variable r=n%5
    variable q=n/5
    while r>1
    bra $+1
r-=2
    endw
    if r==1
    nop
    endif
    if q
    movlw q
    bra $+1
    decfsz WREG
    bra $-2
    endif
    endm

; button selector case    
case macro n, label
    btfsc buttons,n
    bra label
    endm
 
start_timer macro value ; start game timer
    movlw value
    movwf gtimer
    bsf flags, F_GTMR
    endm
    
wait_timer macro ; wait timer expiration
    btfsc flags, F_GTMR
    bra $-1
    endm
    
pause macro value ; suspend execution (busy loop)
    start_timer value
    wait_timer
    endm

#ifdef SOUND_SUPPORT    
wait_sound macro ; wait sound end
    btfsc flags,F_SND
    bra $-1
    endm
#endif
    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; software stack for arguments passing
; SP is Stack Pointer
; T is Top Of Stack
#define SP FSR1
#define T INDF1
STACK_SIZE equ 32
 
S0 equ 0x1F ; stack base address - 1

pstack_init macro ; initialise stack pointer
    clrf FSR1H
    movlw S0
    movwf FSR1L
    endm
    
push macro   ; ( -- n ) push WREG on stack
    movwi ++SP
    endm

pop macro   ; ( n -- )  pop T in WREG
    moviw SP--
    endm
    
drop macro  ; ( n -- ) ; discard T
   addfsr SP,-1
   endm

drop_n macro n  ; discard n elements  n<32
    addfsr SP,-n
    endm
    
lit macro  n ; ( -- n ) ; push a literal
   movlw n
   movwi ++SP 
   endm

dup macro ; ( n -- n n ) duplicate T
    movfw T
    movwi ++SP
    endm
    
over macro ; ( n1 n2 -- n1 n2 n1 )
    moviw -1[SP]
    push
    endm
    
pick macro n  ; pick nth element of stack to WREG
    moviw -n[SP]
    endm

insert macro n ;  insert WREG at position n on stack
   movwi -n[SP]
   endm
   
inc_n macro n ; increament nth element of stack
    moviw -n[SP]
    incf WREG,F
    movwi -n[SP]
    endm
    
dec_n macro n ; decreament nth element of stack
    moviw -n[SP]
    decf WREG,F
    movwi -n[SP]
    endm
    
reserve macro n ; reserve n bytes on stack for local variables
    addfsr SP, n 
    endm
 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; parameter stack section    
dstack udata 0x20
_stack res 32
; game variables section 
GAME_VAR equ 0 ; game variables bank
game_var udata 0x40 
; tetris game state
tetrim res 1 ; active tretriminos 
angle res 1 ; tetriminos rotatation angle in multiple of 90degr. {0:3}
tx  res 1 ; x coordinate
ty  res 1 ; y coordinate
scoreL res 1 ; game score  16 bits
scoreH res 1 
droppedL res 1	; total
droppedH res 1	; dropped lines
  
 
; These 3 sections are used for video pixels buffering
; with indirect access using FSR0
; to form a contiguous address space. 
; total size 192 bytes 
vb_b0    udata 0x50
video_buffer_b0 res 32
vb_b1    udata 0xA0
video_buffer_b1 res 80
vb_b2    udata 0x120
video_buffer_b2 res 80
 
 
    udata_shr
;scan lines counter 16 bits
lcountL res 1
lcountH res 1
; boolean flags 
flags res 1
; game timer
gtimer res 1 
#ifdef SOUND_SUPPORT 
; sound timer
tone_tmr res 1 
#endif 
; lfsr PRNG register
randL res 1 
randH res 1 
; loop counter
rcount res 1
; arithmetic accumulator A 16 bits
accaL res 1
accaH res 1
; arithmetic accumulator B 16 bits
accbL res 1
accbH res 1
; game pad buttons state
buttons res 1
; program memory reader next nibble {0-3}
nibble res 1
; temporary registers
t0  res 1
t1  res 1
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; reset entry point
;;;;;;;;;;;;;;;;;;;;;;;;;;;;
rst: 
    org 0
    banksel ANSELA
    clrf ANSELA
    bsf ANSELA,PAD_PIN
    goto init
    
    org 4
;;;;;;;;;;;;;;;;;;;;;;;;;;;;   
; interrupt service routine
; the only interrupt is on SYNC_PWM timer
; this intterrupt trigger once 
; for every horizontal line, i.e. 15748/sec.    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
isr:
    banksel SYNC_PWMINTF
    bcf SYNC_PWMINTF,PRIF
    btfsc lcountH,0
    bra gt_255 ; lcount > 255
; lcount < 256
    movfw lcountL
    skpnz
    bra vsync_start
    xorlw 3
    skpnz
    bra vsync_end
; check if visible line    
    movfw lcountL
    sublw FIRST_VIDEO-1
    skpnc
    bra isr_exit
    movfw lcountL
    sublw LAST_VIDEO
    skpc
    bra isr_exit
; output video pixels for this line    
    call video_serialize 
    bra isr_exit 
; scanline 0 start vertical sync pulse    
vsync_start: 
    banksel SYNC_PWMDCL
    movlw (HORZ_PERIOD-HORZ_PULSE)&0xff
    movwf SYNC_PWMDCL
    movlw (HORZ_PERIOD-HORZ_PULSE)>>8
    movwf SYNC_PWMDCH
    bsf SYNC_PWMLDCON,LDA
    bsf flags, F_VSYNC
    bra isr_exit
; scanline 3 end vertical sync pulse
vsync_end: 
    banksel SYNC_PWMDCL
    movlw HORZ_PULSE&0xff
    movwf SYNC_PWMDCL
    movlw HORZ_PULSE>>8
    movwf SYNC_PWMDCH
    bsf SYNC_PWMLDCON,LDA
    bcf flags, F_VSYNC
    bra isr_exit
; scan line > 255    
gt_255:
; check for end of field    
    movlw LAST_LINE&0xff
    btfss flags,F_EVEN
    addlw 1
    xorwf lcountL,W
    skpz
    bra tasks
; end of field
; reset line counter    
    clrf lcountL
    clrf lcountH
    movlw 1<<F_EVEN
    xorwf flags,F ; toggle even field flag
    retfie
; round robin task scheduler
; each task execute once every 1/60th sec.
; condition:    
;   Task must complete inside
;   horizontal period. i.e. before SYNC_PWM period end
; maximum 6 tasks slots available    
tasks:
; task selector    
    movfw lcountL
    skpnz
    bra task0
    decf WREG
    skpnz 
    bra task1
    decf WREG
    skpnz
    bra task2
    bra isr_exit
task0:
#ifdef SOUND_SUPPORT    
; sound timer    
    movfw tone_tmr
    skpnz
    bra isr_exit
    decfsz tone_tmr,F
    bra isr_exit
    bcf flags,F_SND
    banksel AUDIO_PWMCON
    bcf AUDIO_PWMCON,EN
#endif    
    bra isr_exit
task1:   
; game timer    
    movf gtimer,F ; 
    skpnz
    bra isr_exit
    decfsz gtimer,F ; 
    bra isr_exit ;
    bcf flags, F_GTMR
    bra isr_exit ;
task2:
; rotate lfsr
    lsrf randH
    rrf randL
    skpnc
    movlw LFSR_TAPS
    xorwf randH
isr_exit:
    incf lcountL
    skpnz
    incf lcountH
    retfie

;div10    
; divsion by 10    
; needed to convert binary to BCD
; input:
;   acca dividend  16 bits
; output:
;   acca: quotient
;   accbL: remainder    
div10:
    movlw 17
    movwf rcount
    clrf accbL
div10_loop:
    movlw 10
    subwf accbL,W
    skpnc
    movwf accbL
    rlf accaL
    rlf accaH
    rlf accbL
    decfsz rcount
    bra div10_loop
    lsrf accbL
    return
    
;read_pad    
;read game pad
; store value in 'buttons'
; a button is accepted if
; the Vadc value is below its threshold
read_pad:
    clrf buttons
    btfsc flags, F_SND
    return ; can't read while tone playing
    banksel TRISA
    bsf TRISA,ADC_PIN
    banksel ADCON0
    bsf ADCON0,ADON
    tcyDelay 5*4 ; d�lais d'acquisition 4�sec
    bsf ADCON0,GO
    btfsc ADCON0,NOT_DONE
    bra $-1
    bcf ADCON0,ADON
; try each button from lower to upper
try_a:
    try_button A_THR, try_b
    bsf buttons,BTN_A
    bra read_exit
try_b:    
    try_button B_THR, try_rt
    bsf buttons,BTN_B
    bra read_exit
try_rt:
    try_button RT_THR, try_up
    bsf buttons,BTN_RT
    bra read_exit
try_up:
    try_button UP_THR, try_lt
    bsf buttons,BTN_UP
    bra read_exit
try_lt:
    try_button LT_THR, try_dn
    bsf buttons,BTN_LT
    bra read_exit
try_dn:
    try_button DN_THR, read_exit
    bsf buttons,BTN_DN
read_exit:
    banksel TRISA
    bcf TRISA,ADC_PIN
    return

#ifdef SOUND_SUPPORT    
; play a tone
; input:
;   t   duration in multiple of 1/60 sec.
;   n   note number
tone: ; ( t n -- )
    banksel TRISA
    bcf TRISA,AUDIO_PIN
    banksel AUDIO_PWMPRL
    lslf T
    movfw T
    call tone_pr
    movwf AUDIO_PWMPRL
    lsrf WREG
    movwf AUDIO_PWMDCL
    incf T,W
    call tone_pr
    movwf AUDIO_PWMPRH
    lsrf WREG
    movwf AUDIO_PWMDCH
    skpnc
    bsf AUDIO_PWMDCL,7
    drop  ; n
    pop   ; WREG=t
    movwf tone_tmr
    bsf AUDIO_PWMCON,EN
    bsf AUDIO_PWMLDCON,LDA
    bsf flags,F_SND
    return
#endif
    
; set_yptr    
; set FSR0 to point to
; video_buffer line y    
; input:
;   y  line number {0:YZISE-1}
; output:
;   FSR0 point to beginning of line y    
set_yptr: ; ( y -- )
    movlw VIDEO_BUFFER&0xff
    movwf FSR0L
    movlw VIDEO_BUFFER>>8
    movwf FSR0H
    lslf T,F
    lslf T,W
    addwf T,F
    pop
    addwf FSR0L
    clrw 
    addwfc FSR0H
    return
    
;video_serialize
; serialise video pixels to output pin
; use EUSART in synchronous mode    
; called from isr    
; input:
;   none
; output:
;   none    
video_serialize: 
    tcyDelay HORZ_DLY
; save FSR0    
    movfw FSR0L
    push
    movfw FSR0H
    push
    movlw FIRST_VIDEO
    subwf lcountL,W
    lsrf WREG  ; a pixel is 4 scan lines
    lsrf WREG
    push      ; FSR0L FSR0H y
    call set_yptr ; FSR0L FSR0H
    lit BPL ; FSR0L FSR0H BPL
    banksel RCSTA
    bsf RCSTA,SPEN
byte_serialize:    
    banksel PIXEL_TXREG
    moviw FSR0++
    movwf PIXEL_TXREG
    banksel TX_PIR
    btfss TX_PIR,TXIF
    bra $-1
    decfsz T
    bra byte_serialize
    drop ; discard bytes counter
    banksel PIXEL_TXREG
    clrf PIXEL_TXREG
    banksel TX_PIR
    btfss TX_PIR,TXIF
    bra $-1
    banksel RCSTA
    bcf RCSTA, SPEN
;restore FSR0    
    pop
    movwf FSR0H
    pop
    movwf FSR0L
    return

; clear_screen
; set all pixels black
; input:
;   none
; output:
;   none    
clear_screen: 
    lit 0
    call set_yptr
    movlw BUFFER_SIZE
    push  ; T=byte count  
    movlw 0
cls_loop:    
    movwi FSR0++
    decfsz T
    bra cls_loop
    drop
    return

;set_pixel_ptr    
;set FSR0 pointer at byte containing pixel x,y
; input:
;   x,y coordinates of pixel in video_buffer    
; output:
;   WREG = pixel mask  
; NOTE: EUSART output Least Significant Bit first
;       LSB appear left on screen.
set_pixel_ptr:  ; ( x y -- )
    call set_yptr
; FSR0+x/8
    lsrf T,W   
    lsrf WREG
    lsrf WREG
    addwf FSR0L
    clrw
    addwfc FSR0H
;create bit mask    
    movlw 7
    andwf T,F ; T=bit position
    movlw 0x1
    skpnz 
    bra mask01 ; least significant bit
mask00:
    lslf WREG
    decfsz T
    bra mask00
mask01:
    drop ; discard shift counter
    return

; set_pixel
; set x,y pixel to 1
; input:
;   x,y pixel coordinates x{0:XSIZE-1},y{0:YSIZE-1}
; output:
;   none
set_pixel: ; ( x y -- )
    call set_pixel_ptr
    iorwf INDF0,F
    return

; clear_pixel
; set x,y pixel to 0
; input:
;   x,y pixel coordinates x{0:XSIZE-1},y{0:YSIZE-1}
; output:
;   none
clear_pixel: ; ( x y -- )
    call set_pixel_ptr
    comf WREG
    andwf INDF0,F
    return

; get_pixel    
; return state of pixel
; input:
;   x,y pixel coordinates x{0:XSIZE-1},y{0:YSIZE-1}
; output:
;   WREG=0|2^n  where n{0:7}
;   Z=1 when T==0    
get_pixel: ; ( x y -- )
    call set_pixel_ptr
    andwf INDF0,W
    return
    
; xor_pixel
; inverse pixel at x,y coordinates
; input:
;   x,y pixel coordinates x{0:XSIZE-1},y{0:YSIZE-1}
; output:
;   set F_COLL boolean flag if collision
xor_pixel: ; ( x y -- f )
    call set_pixel_ptr
    xorwf INDF0,F
    andwf INDF0,W
    skpnz
    bsf flags, F_COLL
    return

;xor_row    
; draw row of 4 pixels
; stop when n==0    
; input:
;   {x,y} left coordinates
;   n pixels to draw, 4 bits in high nibble
; output:
;   f=collision flag, return modified value   
; conditions:
;   use 't1' as temporary storage    
; >> no bank dependency << 
xor_row: ; ( n x y -- )
; check for empty row
    pick 2 ; check if n==0
    skpnz
    bra xor_row_done ; row empty nothing to do
    lslf WREG
    insert 2  ; save shifted n
    skpc
    bra xor_row02 ; bit==0 no draw
; bounds check x{0:XSIZE-1}    
    pick 1  ; WREG=x
    btfsc WREG,7 
    bra xor_row02 ; x<0 out of screen
    over ; n x y x 
    over ; n x y x y
    call xor_pixel ; -- n x y
xor_row02:
    inc_n 1  ; x+=1
    bra xor_row
xor_row_done: ; n x y
    drop_n 3  ; clean stack
    return

;get_flash_word    
; read program memory word
; input:
;   ofs offset in table {0:255}    
;   lo is low byte of program address
;   hi is high byte of program address
; output:
;   PMDATH:PMDAL contain 14 bits data word
; side effect:    
;   reset <nibble> variable 
;   modify BSR    
get_flash_word: ; ( ofs lo hi -- )
    banksel PMADR
    pop
    movwf PMADRH
    pop
    movwf PMADRL
    pop
    addwf PMADRL,F
    clrw 
    addwfc PMADRH,F
read_flash:    
    bcf PMCON1,CFGS
    bsf PMCON1, RD
    nop
    nop
    clrf nibble
    return

;next_flash_word    
; increment PMADR 
; and read next flash word    
; input:
;   none
; output:
;   PMDAT    
; side effect:
;   clear 'nibble'    
;   modify BSR
next_flash_word:
    banksel PMADR
    incf PMADRL
    skpnz
    incf PMADRH
    bra read_flash

;get_nibble    
;get nibble from PMDAT
; input:
;   none    
; output:
;   T=nibble in bits 7:4
; side effect:    
;   increment 'nibble' variable    
;   modify BSR
get_nibble: ; ( -- nibble )
    movlw 3
    xorwf nibble,W
    skpnz 
    call next_flash_word
    banksel PMDAT
; select nibble in word    
    movfw nibble
    skpnz
    bra row0
    decfsz WREG
    bra row2
row1:    
    movfw PMDATL
    bra nibble_mask
row2:
    swapf PMDATL,W
    bra nibble_mask
row0:    
    swapf PMDATH,W
nibble_mask:
    andlw 0xf0
    push
    incf nibble,F ; advance nibble counter
    return

;print_row    
; print 4 pixels row
; inputs:
;   x,y left coordinates
; output:    
;   advance y coordinate to next row    
print_row: ; ( x y -- x y )     
    call get_nibble ; x y -- x y n 
    pick 2 ; x
    push    ; x y n x
    pick 2  ; y
    push      ; x y n x y
    call xor_row ; x y n x y -- x y
    incf T,F ; y+=1
    return

;print_char     
; draw character in video_buffer 
; input:
;   x,y left/top coordinates
;   c   character to print 
; output:
;   none    
print_char: ; ( x y c -- )
    lslf T ; 2 words per table entry
    lit low(digits)
    lit high(digits) ; x y ofs adrL adrH
    call get_flash_word  ; -- x y 
    call print_row
    call print_row
    call print_row 
    call print_row 
    call print_row 
    drop_n 2  ; ( x y -- )
    return

;print_tetrim    
; print current tetriminos
; input:
;   none
; use game variables   
;   titrim: tetriminos id
;   angle: rotation angle    
;   tx,ty  left/top coordinates
; output:
;   f boolean collision flag   
; side effect:    
;   modify BSR
print_tetrim: ; ( -- f )
    bcf flags, F_COLL
    banksel GAME_VAR
    movfw angle
    push    
    lslf tetrim,W   ; 4 words per table entry, one for each angle
    lslf WREG
    xorlw 24
    skpnz
    lslf T,F
    xorlw 24
    addwf T,F  ; ofs
    movlw low(tetriminos)
    push    ; ofs addrL
    movlw high(tetriminos)
    push    ; ofs addrL addrH
    call get_flash_word  ; of addrL addrH --
    banksel GAME_VAR
    movfw tx
    push    ; -- x
    movfw ty
    push    ; -- x y 
    call print_row ; x y -- x y+1
    call print_row
    call print_row
    banksel PMDAT
    movlw 0x30
    andwf PMDATH,W
    skpnz
    call print_row
    drop_n 1
    clrf T
    btfsc flags, F_COLL
    incf T,F
    return

;print_label    
;print a text line stored in program memory
; text terminated by 255
; input: 
;  x,y left coordinates
;  PCLH= high(table)
;  T=low(table) 
; output:
;   none
print_label: ; ( x y tableL -- )
    movfw T     ; WREG= labelL
    callw      
    btfsc WREG,7
    bra prt_lbl_done
    push ; x y labelL c
    pick 3  ; WREG=x
    push    ; x y labelL c x 
    pick 3  ; WREG=y
    push    ; x y labelL c x y
    pick 2  ; WREG = c
    push    ; x y labelL c x y c
    call print_char ; x y labelL c x y c -- x y labelL c
    drop    ; x y labelL
    movlw 1
    addwf T,F ; advance to next character in table
    clrw
    addwfc PCLATH,F
    pick 2    ; x
    addlw 4   ; x+=4
    insert 2  
    bra print_label
prt_lbl_done:
    drop_n 3
    return
    
;print_int
;print an integer
;from right to left ( least significant digit first)
; input:    
;   y  line number 
;   number to print in acca    
; output:
;   none
; condition:    
;   accbL contain digit after div10 call    
print_int: ; ( y -- )
    dup	    ; y y
    movlw XSIZE-5
    insert 1	; x y
print_int00:    
    movfw accaH
    iorwf accaL,W
    skpnz
    bra print_last
    over    ; x y x 
    over    ; x y x y
    call div10
    movfw accbL ; remainder of division
    push   ; x y x y d
    call print_char ; -- x y 
    pick 1  ; x
    push    ; x y x
    movlw 4
    subwf T,F ; x y x-4  next position in video_buffer
    pop
    insert 1  ; save that x position
    bra print_int00
print_last:
    pick 1  ; x 
    xorlw XSIZE-5 ; x at right margin?
    skpz
    bra no_zero
    push          ; yes print a zero
    bra print_char
no_zero:
    drop_n 2
    return
    
; draw horizontal line ( length  y x -- )
; inputs:
;   length of line
;   {x,y} left coordinates 
; output:
;   none    
hline:
    pick 2  ; length
    skpnz
    bra hline_done  ; length=0
    decf WREG
    insert 2  ; decrement length
    dup	    ; len y x x
    pick 2  ; y
    push    ; len y x x y
    call xor_pixel
    incf T  ; len y x+1
    bra hline
hline_done:    
    drop_n 3
    return

; vline    
;draw vertical line ( length x y -- )
; inputs:
;   length of line
;   {x,y} top coordinates
; output:
;   none    
vline:  ; ( len x y -- ) 
    pick 2 ; len
    skpnz
    bra vline_done ; len=0
    decf WREG
    insert 2  ; decrement len
    over     ; l x y x
    over     ; l x y x y    
    call xor_pixel ; -- l x y
    incf T,F	; y+=1
    bra vline
vline_done:
    drop_n 3
    return
    
WELL_WIDTH equ 10
WELL_DEPTH equ 22 
;game_init
; initialize game state
; input:
;   none
; output:
;   none    
game_init:
    call clear_screen
; draw game well
    movlw WELL_DEPTH
    push
    push  ; len len
    clrw
    push
    push  ; len len x=0, y=0
    call vline ; -- len 
    lit WELL_WIDTH+1 ; len x
    lit 0	     ; len x y
    call vline
    lit WELL_WIDTH+2
    lit WELL_DEPTH
    lit 0
    call hline
; print "SCORE" label    
    lit WELL_WIDTH+3 ; x
    lit 0	     ; y
    movlp high(lbl_score)
    lit	low(lbl_score)  
    call print_label
; print "LINES" label    
    lit WELL_WIDTH+3 ; x
    lit 12	     ; y
    movlp high(lbl_lines)
    lit low(lbl_lines)
    call print_label
; variables initialization
    bsf flags,F_GSTOP
clear_score:
    banksel GAME_VAR
    clrf scoreL
    clrf scoreH
    clrf droppedL
    clrf droppedH
    return

; drop 1 row
; input:
;   r  row to drop
; output:
;   none
drop1: ; ( r -- )
    reserve 1 ; r x  
row_loop: ; y{r:1}
    movlw 10
    movwf T ; -- r x
col_loop:  ; x{10:1}
    dup	  ; -- r x x
    pick 2
    decf WREG
    push  ; -- r x x r-1
    call get_pixel ; -- r x p
    movwf t0
    dup		; r x x
    pick 2
    push	; r x x r
    movf t0,F
    skpz
    bra drop00
    call clear_pixel ;  r x x r -- r x
    bra next_pixel
drop00:
    call set_pixel ; r x x r -- r x
next_pixel:    
    decfsz T	; r x-1 
    bra col_loop
    dec_n 1  ; dec r
    skpz 
    bra row_loop
    drop_n 2
    return
    
    
ROW_EMPTY equ 0     
ROW_FULL equ 10 ; 10 bits in row
; query_row 
; check the state of well row
; input:
;   s number of bits set initialized at 0 by caller    
;   r row number: 0 top, 21 bottom  
; output:    
;   s={0:10}  number of bits set
query_row: ; ( s=0 r -- s=0:10 )
    lit 10 ; s r x   ; check x from  10 to 1
qr00:
    dup   ; s r x x
    pick 2 
    push  ; s r x x y
    call get_pixel ; -- s r x
    skpnz   ; Z modified by get_pixel
    bra qr01
    inc_n 2
qr01:    
    decfsz T
    bra qr00
    drop_n 2 ; -- s
    return

; count and drop full rows
; input:
;   none
; output:
;   n number of droppend rows
;condition:
;  check each row from bottom up
;  stop at first empty row
;  full rows are dropped    
count_full: ; ( -- n)
    lit 0 ; full rows counter
    lit 21 ; row number {21:1}
count_loop:
    lit 0   ; n r s  number of bits set
    over    ; n r s r
    call query_row ; n r s r -- n r s
    pop	    ; n r
    skpnz
    bra count_done
    xorlw ROW_FULL
    skpz
    bra next_row
; this is a full row    
    inc_n 1 ; increment n
    dup    ; n r r
    call drop1 ; n r r -- n r
    incf T,F
next_row:
    decfsz T,F
    bra count_loop
count_done:
    drop   ; n r -- n
    return

;new_tminos    
; generate a new tetriminos
; input:
;   none
; output:
;   none
new_tminos:
    banksel GAME_VAR
    movlw 7
    andwf randL,W
    xorlw 7
    skpz
    xorlw 7
    movwf tetrim
    movlw 3
    andwf randH,W
    movwf angle
    movlw 4
    movwf tx
    clrf ty
    call print_tetrim ; -- f
    call print_tetrim ; -- f f
    drop
    pop
    skpz
    bsf flags, F_GSTOP
    return

;wait_start    
; wait player start signal
; button A pressed    
; input:
;   none
; output:
;   none
wait_start:
#ifdef SOUND_SUPPORT    
    clrw
    call korobeiniki
    push    ; count
    lit 0   ; sequence
koro:
    incf T,F ; -- count sequence
    movfw T  ; WREG=sequence
    call korobeiniki
    push    ; -- count sequence duration
    inc_n 1 ;
    pick 1  
    call korobeiniki
    push    ; -- count nidx duration note
    call tone  ; -- count nidx 
    wait_sound
    call read_pad
    btfsc buttons, BTN_A
    bra wait_end
    dec_n 1
    skpz
    bra koro
    drop_n 2
    pause 60
    bra wait_start
wait_end:    
    drop_n 2
#else
    pause 20
    call read_pad
    btfss buttons, BTN_A
    bra wait_start
#endif    
    call clear_score
    bcf flags, F_GSTOP
    return
 
;coll_test
; collision test
; after rotation or translation
; undo last action if collision    
; input:
;   none
; output:
;   none
coll_test: 
    call print_tetrim ; -- f
    call print_tetrim ; -- f f
    drop
    pop
    skpnz
    return
; collision occured undo last move    
    banksel GAME_VAR
;    movfw buttons
    case BTN_B, undo_drop_tetrim
    case BTN_UP, undo_rot_right
    case BTN_DN, undo_rot_left
    case BTN_RT, undo_move_right
    case BTN_LT, undo_move_left
    return
undo_drop_tetrim:
    decf ty,F
    return
undo_rot_right:
    decf angle,F
    movlw 3
    andwf angle,F
    return
undo_rot_left:
    incf angle,F
    movlw 3
    andwf angle,F
    return
undo_move_right:
    decf tx,F
    return
undo_move_left:
    incf tx,F
    return

;update_display    
; print 'score' and 'dropped' values
; input:
;   none
; output:
;   none
update_display:    
; print score
    banksel GAME_VAR
    movfw scoreL
    movwf accaL
    movfw scoreH
    movwf accaH
    lit 6
    call print_int
;print dropped line
    banksel GAME_VAR
    movfw droppedL
    movwf accaL
    movfw droppedH
    movwf accaH
    lit 18
    call print_int
    return

;game_over
; signal game terminated
; input:
;   none
; output:
;   none
game_over:
    lit 10
    lit 0
clr00:    
    over
    over
    call clear_pixel ; x y x y -- x y
    dec_n 1 ; x-=1
    skpz
    bra clr00
    movlw 21
    movwf T
    insert 1  ; counter 21
drop21:
    dup    ; counter 21 21
    call drop1 ; -- counter 21
    pause 6
    pick 1
    decf WREG
    insert 1  ; -- counter-1 21
    skpz
    bra drop21
    drop_n 2
    return

;prompt
; print prompt message    
prompt:
    lit 0
    lit 24
    movlp high(lbl_press)
    lit low(lbl_press)
    call print_label
    return
    
;;;;;;;;;;;;;;;;;;;;;;;;;
;   game logic
;;;;;;;;;;;;;;;;;;;;;;;;;    
tetris:
    call update_display
;print start prompt
    call prompt
; wait button A press
; to start game    
    call wait_start 
; delete prompt
    call prompt
; game start here
game_loop:
; generate new tetriminos
; if collision at this stage
; game is over
    call new_tminos
    btfss flags, F_GSTOP
    bra fall_loop
    call game_over
    call game_init
    bra tetris
fall_loop: ; tetriminos fall in the well
    call print_tetrim
    pop
    pause 10
    call print_tetrim ; erase tetriminos
    pop
; read pad
    call read_pad
    banksel GAME_VAR
    case BTN_B, drop_tetrim
    case BTN_UP, rot_right
    case BTN_DN, rot_left
    case BTN_RT, move_right
    case BTN_LT, move_left
    bra move_down
drop_tetrim:
    banksel GAME_VAR
    incf ty,F
    call coll_test
    btfss flags, F_COLL
    bra $-4
    call print_tetrim
    pop
    bra score_update
rot_left:
    decf angle,F
    movlw 3
    andwf angle,F
    call coll_test
    bra move_down
rot_right:
    incf angle,F
    movlw 3
    andwf angle,F
    call coll_test
    bra move_down
move_left:
    decf tx,F
    call coll_test
    bra move_down
move_right:
    incf tx,F
    call coll_test
; move down
move_down:
    bcf flags, F_COLL
    banksel GAME_VAR
    incf ty,F ; tetriminos fall
    call print_tetrim ; -- f
    call print_tetrim ; -- f f
    drop
    pop
    skpnz
    bra fall_loop
    banksel GAME_VAR
    decf ty,F
    call print_tetrim
    pop
score_update:    
; check full row and clean
    call update_display ; erase numbers
    call count_full
; add full row count to dropped variable
    banksel GAME_VAR
    movfw T
    addwf droppedL,F ; 
    clrw
    addwfc droppedH
; count points = 2^n where n is count of dropped rows {1:4}    
    movf T,F
    skpnz
    bra add_points
    movlw 1
    lslf WREG
    decfsz T,F
    bra $-2
add_points:
    addwf scoreL
    clrw
    addwfc scoreH
    drop 
    call update_display ; display new values
    bra game_loop
    return

;;;;;;;;;;;;;;;;;;;;;;;;;;
; hardware initialization
;;;;;;;;;;;;;;;;;;;;;;;;;;    
init:
    pstack_init
; ADC configuration
    banksel ADCON0
    movlw (PAD_CHS<<CHS0)
    movwf ADCON0
    movlw (2<<ADCS0)
    movwf ADCON1
    banksel WPUA
    bcf WPUA, PAD_PIN
;;;;;;;;;;;;;;;;;;;;;    
    banksel TRISA
    movlw ~((1<<SYNC_OUT)|(1<<VIDEO_OUT)|(1<<LED_PIN))
    andwf TRISA,F
    banksel VIDEO_LAT
    bcf VIDEO_LAT,VIDEO_OUT
    bcf LATA,LED_PIN
#ifdef SOUND_SUPPORT    
; audio pwm initialization
    bcf LATA,AUDIO_PIN
    banksel PWM2PH
    clrf PWM2PHL
    clrf PWM2PHH
    clrf PWM2OFL
    clrf PWM2OFH
    bsf AUDIO_PWMCON,OE
    banksel APFCON
    bsf APFCON,P2SEL
#endif   
;configure EUSART in sychronsous mode
;to use as pixel serializer
    banksel SPBRG
    movlw 2
    movwf SPBRGL
    clrf SPBRGH
    clrf TXREG
    movlw (1<<SYNC)|(1<<CSRC)|(1<<TXEN)
    movwf TXSTA
    bsf RCSTA, SPEN
    movlw ~((1<<CREN)|(1<<SREN))
    andwf RCSTA
; configure pwm video sync for horizontal period
    banksel SYNC_PWMDCH
    clrf SYNC_PWMPH
    clrf SYNC_PWMPH+1
    clrf SYNC_PWMLDCON
    movlw HORZ_PERIOD&0XFF
    movwf SYNC_PWMPRL
    movlw HORZ_PERIOD>>8
    movwf SYNC_PWMPRH
    clrf SYNC_PWMCLKCON
    movlw (3<<OE)|(1<<POL)
    movwf SYNC_PWMCON
    movlw HORZ_PULSE&0xff
    movwf SYNC_PWMDCL
    movlw HORZ_PULSE>>8
    movwf SYNC_PWMDCH
    bsf SYNC_PWMLDCON,LDA
   ;enable video interrupt on period match
    bcf SYNC_PWMINTF,PRIF
    bsf SYNC_PWMINTE,PRIE
    ; enable peripheral interrupt
    banksel SYNC_PIE
    bsf SYNC_PIE,SYNC_IE
    ;enable interrupts
    movlw (1<<GIE)|(1<<PEIE)
    movwf INTCON
    clrf lcountL
    clrf lcountH
    clrf flags
    ; seed lfsr PRNG
    movlw 0xAC
    movwf randL
    movlw 0xE1
    movwf randH


;;;;;;;;;;;;;;;;;;;;;
;  main function
;;;;;;;;;;;;;;;;;;;;;    
main:
    call game_init
    call tetris
    bra main
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;   data tables
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; **************** encoding ************************************************
; enhanced PIC instructions are coded on 14 bits
; these 14 bits can be read as data using program memory
; writing/reading mechanism, see datasheet chapter 10 for detail.
; the 'dw' assembler directive encode data in those 14 bits contrary to 'dt'
; directive which encode a single byte as a RETLW instruction.
; So we get a 50% compression by encoding 3 nibbles/program word. 
; The encoding used in the following tables is as follow:    
; bits 13:12  number of nibbles where 0 means there is more than 3 nibbles
; in this data chunk so 1 or more other words follow this one.    
; other 12 bits are 3 nibbles (i.e. 4 bits slices).
;****************************************************************************

digits: ; each digit is 5 rows
    dw 0x0EAA,0x2AE0 ; 0
    dw 0x04C4,0x24E0 ; 1
    dw 0x0E2E,0x28E0 ; 2
    dw 0x0E2E,0x22E0 ; 3
    dw 0x0AAE,0x2220 ; 4
    dw 0x0E8E,0x22E0 ; 5
    dw 0x088E,0x2AE0 ; 6
    dw 0x0E22,0x2220 ; 7
    dw 0x0EAE,0x2AE0 ; 8
    dw 0x0EAE,0x2220 ; 9
; letters needed for 'SCORE' and 'LINES" labels
    dw 0x0EAE,0x2AA0 ; A  code 10
    dw 0x0688,0x2860 ; C  code 11
    dw 0x0E8E,0x28E0 ; E  code 12
    dw 0x0E44,0x24E0 ; I  code 13
    dw 0x0888,0x28E0 ; L  code 14
    dw 0x08CA,0x2AA0 ; N  code 15
    dw 0x04AA,0x2A40 ; O  code 16
    dw 0x0EAE,0x2880 ; P  code 17
    dw 0x08EA,0x2880 ; R  code 18
    dw 0x0684,0x22C0 ; S  code 19
    dw 0x0040,0x2400 ; :  code 20
    dw 0x2000,0x2000 ; space code 21

; comments notation: Rn clockwise rotation n*90 degr.    
; note that vertical I as 4 rows so it needs 2 words    
tetriminos: 
    dw 0x388C ; L R0
    dw 0x30E8 ; L R1
    dw 0x3C44 ; L R2
    dw 0x32E0 ; L R3
    dw 0x344C ; J R0
    dw 0x38E0 ; J R1
    dw 0x3644 ; J R2
    dw 0x30E2 ; J R3
    dw 0x3CC0 ; O R0 
    dw 0x3CC0 ; O R2 
    dw 0x3CC0 ; O R2 
    dw 0x3CC0 ; O R3 
    dw 0x36C0 ; S R0
    dw 0x3462 ; S R1
    dw 0x36C0 ; S R2
    dw 0x3462 ; S R3
    dw 0x3E40 ; T R0
    dw 0x3262 ; T R1
    dw 0x304E ; T R2
    dw 0x38C8 ; T R3
    dw 0x3C60 ; Z R0
    dw 0x34C8 ; Z R1
    dw 0x3C60 ; Z R3
    dw 0x34C8 ; Z R4
; annoying! I tetriminos need a special treatment 
; because vertical I need 2 words for encoding.   
    dw 0x0444,0x1400 ; I R0  
    dw 0x300F,0x1000 ; I R1  second word is filling for alignment
    dw 0x0222,0x1200 ; I R2
    dw 0x20F0,0x1000 ; I R3  

#ifdef SOUND_SUPPORT
; tempered scale timer period values    
tone_pr:
    brw
    dt 0x02, 0xed   ; 330   (MI4)   0
    dt 0xb5, 0xdf   ; 349   (FA4)   1
    dt 0x28, 0xd3   ; 370   (FA4#)  2
    dt 0x4c, 0xc7   ; 392   (SOL4)  3
    dt 0x1e, 0xbc   ; 415   (SOL4#) 4
    dt 0x8f, 0xb1   ; 440   (LA4)   5
    dt 0x98, 0xa7   ; 466   (LA4#)  6
    dt 0x30, 0x9e   ; 494   (SI4)   7
    dt 0x4f, 0x95   ; 523hz (DO5)   8
    dt 0xed, 0x8c   ; 554   (DO5#)  9
    dt 0x04, 0x85   ; 587   (R�5)   10
    dt 0x8d, 0x7d   ; 622   (R�5#)  11
    dt 0x81, 0x76   ; 659   (MI5)   12
    dt 0xda, 0x6f   ; 698   (FA5)   13
    dt 0x93, 0x69   ; 740   (FA5#)  14
    dt 0xa6, 0x63   ; 784   (SOL5)  15
    dt 0x0f, 0x5e   ; 831   (SOL5#) 16
    dt 0xc7, 0x58   ; 880   (LA5)   17
    dt 0xcc, 0x53   ; 932   (LA5#)  18
    dt 0x18, 0x4f   ; 988   (SI5)   19
    dt 0x82, 0x42   ; 1046hz (do6)  20
    dt 0, 0	    ; silence	    21

    
; a russian folklore soung    
korobeiniki:
    ;nombre de notes
    brw
    dt 34
    ; dur�, note
    dt 45, 0 
    dt 15, 4
    dt 30, 7
    dt 15, 3
    dt 15, 0
    
    dt 45, 5
    dt 15, 9
    dt 30, 12
    dt 15, 10
    dt 15, 8
    
    dt 45, 7
    dt 15, 8
    dt 30, 10
    dt 30, 12
    
    dt 30, 8
    dt 30, 5
    dt 60, 5
    
    dt 45, 13
    dt 15, 15
    dt 30, 17
    dt 15, 15
    dt 15, 13
    
    dt 45, 12
    dt 15, 13
    dt 30, 12
    dt 15, 10
    dt 15, 8
    
    dt 45, 7
    dt 15, 8
    dt 30, 10
    dt 30, 12
    
    dt 30, 8
    dt 30, 5
    dt 60, 5
    
 #endif

lbl_score: ; "SCORE" label
    dt 19,11,16,18,12,20,255
    
lbl_lines: ; "LINES"  label
    dt 14,13,15,12,19,20,255

lbl_press: ; "PRESS A" label
    dt 17,18,12,19,19,21,10,255

    
 
    end

    