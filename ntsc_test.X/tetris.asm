    include p12f1572.inc
    
    radix dec
    errorlevel 0, -302, -305
    
    __config _CONFIG1, _FOSC_ECH&_WDTE_OFF
    __config _CONFIG2, _PLLEN_OFF&_LVP_OFF 

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
 
AUDIO_PIN equ RA4
AUDIO_PWMDCL equ PWM2DCL
AUDIO_PWMDCH equ PWM2DCH
AUDIO_PWMPRL equ PWM2PRL
AUDIO_PWMPRH equ PWM2PRH
AUDIO_PWMTMR equ PWM2TMR
AUDIO_PWMCLKCON equ PWM2CLKCON
AUDIO_PWMLDCON equ PWM2LDCON
AUDIO_PWMCON equ PWM2CON
 
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
F_MUTEX equ 2 ; video routine lock out
F_GTMR equ 3  ; game timer active
F_GSTOP equ 4 ; game stopped
F_SND equ 5 ; sound timer active 
 
LFSR_TAPS equ 0xB4 ; xor mask
 
disable_video macro ; lockout video_serialize
    bsf flags, F_MUTEX
    endm
    
enable_video macro ; unlock video_serialize
    bcf flags, F_MUTEX
    endm
    
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

case macro n, label
    xorlw n
    skpnz
    bra label
    xorlw n
    endm
 
ldpmadr macro addr ; load PMADR register
    movlw low(addr)
    movwf PMADRL
    movlw high(addr)
    movwf PMADRH
    endm
    
addpmadr macro value ; add value to PMADR
    movlw value
    addwf PMADRL
    clrw
    addwfc PMADRH
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
 
wait_sound macro ; wait sound end
    btfsc flags,F_SND
    bra $-1
    endm
    
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
    
swap macro ; exchange WREG and T
    xorwf T
    xorwf T,W
    xorwf T
    endm
 
store macro var ; ( n -- ) pop T in variable
   moviw SP--
   movwf var
   endm
   
fetch macro var  ; ( -- n ) push var on stack
   movfw var
   movwi ++SP
   endm

pick macro n  ; pick nth element of stack to WREG
    moviw -n[SP]
    endm

insert macro n ;  insert WREG at position n on stack
   movwi -n[SP]
   endm
   
movtw macro ; ( n -- n ) overwrite WREG with T
   movfw T
   endm
   
movwt macro ; ( n1 -- n2 ) overwrite T with WREG
    movwf T
    endm
    
add macro  ; add WREG to T result on T
    addwf T,F
   endm
   
sub macro  ; substract WREG from T result on T
   subwf T,F
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
    
zbranch macro label ; branch if T == 0
    pop
    skpnz
    bra label
    endm
    
tbranch macro label ; branch if T != 0
    pop
    skpz
    bra label
    endm
    
reserve macro n ; reserve n bytes on stack for local variables
    addfsr SP, n 
    endm
 
rot macro ; ( n1 n2 n3 -- n2 n3 n1 )
    pick 2
    push	; n1 n2 n3 n1
    pick 2
    insert 3   ;  n2 n2 n3 n1
    pick 1
    insert 2   ; n2 n3 n3 n1
    pop        ; n2 n3 n3 
    movwf T    ; n2 n3 n1
    endm
 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; parameter stack section    
dstack udata 0x20
_stack res 32
; game variables section 
GAME_VAR equ 0 ; variables bank
game_var udata 0x40 
; tetris game state
tetrim res 1 ; active tretriminos 
angle res 1 ; tetriminos rotatation angle in multiple of 90degr. {0:3}
tx  res 1 ; x coordinate
ty  res 1 ; y coordinate
scoreL res 1 ; game score  16 bits
scoreH res 1 
dropped res 1 ; dropped lines

; These 3 sections are used for video pixels buffering
; with indirect access using FSR0
; to form a contiguous address space. 
vb_b0    udata 0x50
video_buffer_b0 res 32
vb_b1    udata 0xA0
video_buffer_b2 res 80
vb_b2    udata 0x120
video_buffer_b3 res 80
 
 
    udata_shr
;scan lines counter 16 bits
lcountL res 1
lcountH res 1
; boolean flags 
flags res 1
; game timer
gtimer res 1 
; sound timer
tone_tmr res 1 
; lfsr PRNG register
randL res 1 
randH res 1 
; loop counter
rcount res 1
; arithmetic accumulator A 24 bits
accaL res 1
accaH res 1
; arithmetic accumulator B 16 bits
accbL res 1
accbH res 1
; game pad buttons state
buttons res 1
; flash reader next nibble {0-3}
nibble res 1
 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
rst: 
    org 0
    banksel ANSELA
    clrf ANSELA
    bsf ANSELA,PAD_PIN
    goto init
    
    org 4
isr:
    banksel SYNC_PWMINTF
    bcf SYNC_PWMINTF,PRIF
    btfsc lcountH,0
    bra gt_255 ; lcount > 255
    movfw lcountL
    skpnz
    bra vsync_start
    xorlw 3
    skpnz
    bra vsync_end
lt_256: ; lcount < 256
    btfsc flags, F_MUTEX
    bra isr_exit
    movfw lcountL
    sublw FIRST_VIDEO-1
    skpnc
    bra isr_exit
    movfw lcountL
    sublw LAST_VIDEO
    skpc
    bra isr_exit
    call video_serialize
    bra isr_exit    
vsync_start: ; scanline 0 start vertical sync pulse
    banksel SYNC_PWMDCL
    movlw (HORZ_PERIOD-HORZ_PULSE)&0xff
    movwf SYNC_PWMDCL
    movlw (HORZ_PERIOD-HORZ_PULSE)>>8
    movwf SYNC_PWMDCH
    bsf SYNC_PWMLDCON,LDA
    bsf flags, F_VSYNC
    bra isr_exit
vsync_end: ; scanline 3 end vertical sync pulse
    banksel SYNC_PWMDCL
    movlw HORZ_PULSE&0xff
    movwf SYNC_PWMDCL
    movlw HORZ_PULSE>>8
    movwf SYNC_PWMDCH
    bsf SYNC_PWMLDCON,LDA
    bcf flags, F_VSYNC
    bra isr_exit
gt_255: ; lcount>255
    movlw LAST_LINE&0xff
    btfss flags,F_EVEN
    addlw 1
    xorwf lcountL,W
    skpz
    bra tasks
    clrf lcountL
    clrf lcountH
    movlw 1<<F_EVEN
    xorwf flags,F
    retfie
; round robin task scheduler
; each task execute once every 1/60th sec.
; condition:    
;   Task must complete inside
;   horizontal period.
tasks:
    movfw lcountL
    case 0, task0
    case 1, task1
    case 2, task2
    bra isr_exit
task0:
; sound timer    
    movfw tone_tmr
    skpnz
    bra isr_exit
    decfsz tone_tmr,F
    bra isr_exit
    bcf flags,F_SND
    banksel AUDIO_PWMCON
    bcf AUDIO_PWMCON,EN
    banksel TRISA
    bsf TRISA,AUDIO_PIN
    bra isr_exit
task1:   
; decrement game timer    
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
    bra isr_exit 
isr_exit:
    incf lcountL
    skpnz
    incf lcountH
    retfie

;divsion by 10    
;needed to convert binary to BCD
; input:
;   acca dividend  16 bits
; output:
;   acca: quotient
;   accbL: remainder    
div10:
    disable_video
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
    enable_video
    return
    
    
; WREG*6  because BPL=6
; WREG*6=WREG*4+WREG*2    
mult6: 
    movwf accbH
    lslf accbH    ; accbH=2*WREG
    lslf accbH,W  ; WREG=4*WREG
    addwf accbH,W ; WREG=6*WREG
    return

;read game pad
; store value in
; buttons
; a button is accepted if
; the Vadc value is below its threshold
read_pad:
    clrf buttons
    btfsc flags, F_SND
    return ; can't read while tone playing
    banksel ADCON0
    bsf ADCON0,ADON
    tcyDelay 8*4 ; d�lais d'acquisition 4�sec
    bsf ADCON0,GO
    btfsc ADCON0,NOT_DONE
    bra $-1
    bcf ADCON0,ADON
; try each button from lower to upper
try_a:
    try_button A_THR, try_b
    bsf buttons,BTN_A
    return
try_b:    
    try_button B_THR, try_rt
    bsf buttons,BTN_B
    return
try_rt:
    try_button RT_THR, try_up
    bsf buttons,BTN_RT
    return
try_up:
    try_button UP_THR, try_lt
    bsf buttons,BTN_UP
    return
try_lt:
    try_button LT_THR, try_dn
    bsf buttons,BTN_LT
    return
try_dn:
    try_button DN_THR, no_button
    bsf buttons,BTN_DN
no_button:
    return
    
; play a tone
; input:
;   t   duration in multiple of 1/60 sec.
;   n   note number
tone: ; ( t n -- )
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
    pop
    pop
    movwf tone_tmr
    bsf AUDIO_PWMCON,EN
    bsf AUDIO_PWMLDCON,LDA
    bsf flags,F_SND
    banksel TRISA
    bcf TRISA,AUDIO_PIN
    return
    
    
; serialise video pixels to scan line
; due to timing constrain use specialized 
; division and multiplication    
video_serialize: 
    movfw FSR0L
    push
    movfw FSR0H
    push
    lit 0 ; ( x )
    tcyDelay HORZ_DLY
    movlw VIDEO_BUFFER&0xff
    movwf FSR0L
    movlw VIDEO_BUFFER>>8
    movwf FSR0H
    movlw FIRST_VIDEO
    subwf lcountL,W
    lsrf WREG
    lsrf WREG
    call mult6
    addwf FSR0L
    clrw
    addwfc FSR0H
    movlw XSIZE/8
    movwf rcount
    banksel RCSTA
    bsf RCSTA,SPEN
pixels_loop:    
    moviw FSR0++
    banksel PIXEL_TXREG
    movwf PIXEL_TXREG
    banksel TX_PIR
    btfss TX_PIR,TXIF
    bra $-1
    decfsz rcount
    bra pixels_loop
    banksel PIXEL_TXREG
    clrf PIXEL_TXREG
    banksel TX_PIR
    btfss TX_PIR,TXIF
    bra $-1
    banksel RCSTA
    bcf RCSTA, SPEN
    pop
    movwf FSR0H
    pop
    movwf FSR0L
    return

; fill_buffer
; fill screen buffer with WREG value    
fill_buffer: ; ( c -- )
    disable_video
    movlw high(VIDEO_BUFFER)
    movwf FSR0H
    movlw low(VIDEO_BUFFER)
    movwf FSR0L
    movlw BUFFER_SIZE
    push    ; c size
    pick 1
fill_loop:    
    movwi FSR0++
    decfsz T
    bra fill_loop
    drop_n 2
    enable_video
    return

    
;set video pointer at 
; byte that contain pixel {x,y}
; output:
;   T = pixel offset, 0 left, 7 right  
;   Z=0 if offset==0 
; NOTE: EUSART output least significant bit first
;       so bit 0 appear left on screen.    
set_video_ptr:  ; ( x y -- b )
    movlw high(VIDEO_BUFFER)
    movwf FSR0H
    movlw low(VIDEO_BUFFER)
    movwf FSR0L
; buffer_addr+y*BPL
    pop		; ( x y -- x )
    call mult6
    addwf FSR0L
    clrw
    addwfc FSR0H
; buffer_addr+x/8
    lsrf T,W   
    lsrf WREG
    lsrf WREG
    addwf FSR0L
    clrw
    addwfc FSR0H
    movlw 7
    andwf T,F
    return

; return state of pixel
; input:
;   x,y coordinates of pixel
; output:
;   T=0|2^n  where n{0:7}
;   Z reflect content of T    
get_pixel: ; ( x y -- 0|2^n ) n{0:7)
    call set_video_ptr ; x y -- shift_counter
    movlw 1
    skpnz   ; Z modified by set_video_ptr
    bra gpx01
gpx00:
    lslf WREG
    decfsz T
    bra gpx00
gpx01:
    andwf INDF0,W
    movwf T  ; 
    return
    
    
XOR_OP equ 0
CLR_OP equ 1
;operation on pixel    
; inputs:
;   {x,y} coordinates
;   op operation XOR_OP|CLR_OP 
; output:
;   WREG=collision flag, 0 if collision 
; >> no bank dependency << 
bitop: ; ( x y op -- f )
    disable_video
    pick 2
    push	; x y op x
    pick 2
    push	; x y op x y
; preserve FSR0    
    movfw FSR0L
    insert 4
    movfw FSR0H
    insert 3	; -- FSR0L FSR0H op x y 
    call set_video_ptr  ; -- FSR0L FSR0H op bit
    movlw 0x1
    skpnz ; bit Z modified before leaving set_video_ptr
    bra bitop01 ; least significant bit
bitop00:
    lslf WREG
    decfsz T
    bra bitop00
bitop01: 
    ; WREG= bit mask
    movwf T    ; -- FSR0L FSR0H op mask
    pick 1  ; WREG= op
    skpnz
    bra xor_bit  ; 
clear_bit:
    comf T,W
    andwf INDF0,F
    bra bitop02
xor_bit:
    movfw T   ; -- FSR0L FSR0H op mask
    xorwf INDF0,F
    andwf INDF0,W ; on screen bit value = 0 if collision
    movwf T  ; -- FSR0L FSR0H op f
bitop02:    
; restore FSR0    
    pick 2  
    movwf FSR0H
    pick 3
    movwf FSR0L
    pop  ; collision flag
    drop_n 3 ; drop parameters frame
    push ; store collision flag
    enable_video
    return

; draw row of 4 pixels
; stop when n==0    
; input:
;   {x,y} left coordinates
;   n pixels to draw
;   f collision flag to be modified    
; output:
;   f=collision flag, return modified value   
; >> no bank dependency << 
xor_row: ; ( f n x y -- f )
    pick 2 ; check if n==0
    skpnz
    bra xor_row_done ; n==0 done
    lslf WREG
    insert 2  ; save shifted n
    skpc
    bra xor_row02 ; bit==0 no draw
    over ; f n x y x 
    over ; f n x y x y
    lit XOR_OP ; f n x y x y op
    call bitop ; -- f n x y f
    pick 4 ; pick flag
    iorwf T
    pop	     ; -- f n x y
    insert 3 ; store modified flag
xor_row02:
    inc_n 1  ; x+=1
    bra xor_row
xor_row_done: ; f n x y
    drop_n 3  ; only keep f
    return

; read flash memory word
; input:
;   lo is low byte of address
;   hi is high byte of address
;   ofs offset in table (limited to 255 )    
; output:
;   PMDATH: PMDAL
;reset <nibble> variable 
; >> modify BSR <<    
get_flash_word: ; ( ofs lo hi -- )
    banksel PMADR
    pop
    movwf PMADRH
    pop
    movwf PMADRL
    pop
    addwf PMADRL
    clrw 
    addwfc PMADRH
read_flash:    
    bcf PMCON1,CFGS
    bsf PMCON1, RD
    nop
    nop
    clrf nibble
    return

; increament pointer 
; and read next flash word    
; >> modify BSR <<    
next_flash_word:
    banksel PMADR
    incf PMADRL
    skpnz
    incf PMADRH
    bra read_flash
    
;get nibble from PMDAT
; output:
;   stack nibble in bits 7:4
; increment nibble variable    
; >> modify BSR <<    
get_nibble: ; ( -- nibble )
    movlw 3
    xorwf nibble,W
    skpnz ; if set all nibbles of this word read
    call next_flash_word
    banksel PMDAT
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
    incf nibble,F ; advance nibble pointer
    return

; print 4 pixels row
; inputs:
;   x,y left coordinates
;   f  collision flag
; output:    
;   advance y coordinate for next row    
print_row: ; ( x y f -- x y f )     
    call get_nibble ; -- x y f n 
    pick 3 
    push    ; x y f n x
    pick 3
    push      ; x y f n x y
    call xor_row ; -- x y f
    inc_n 1 ; y+=1
    return
    
; draw character 
; input:
;   x,y left/top coordinates
;   c   character to print    
print_char: ; ( x y c -- )
    lslf T ; 2 words per digit
    lit low(digits)
    lit high(digits) ; x y ofs adrL adrH
    call get_flash_word  ; x y
    lit 0   ; x y f=0
    call print_row
    call print_row
    call print_row 
    call print_row 
    call print_row 
    drop_n 3  ; ( x y f -- )
    return

; print current tetriminos
; input:
;   arguments in game variables   
;   titrim: tetriminos id
;   angle: rotation angle    
;   tx,ty  left/top coordinates
; output:
;   collision flag,  0 if no collision    
; >> modify BSR <<    
print_tetrim: ; (  -- f )
    banksel GAME_VAR
    lslf tetrim,W
    lslf WREG
    push
    movlw 24
    subwf T,W
    skpnc
    bra ti ; I tetrominos
    movfw angle
    addwf T,F
    movlw low(tetriminos)
    push    ; ofs addrL
    movlw high(tetriminos)
    push    ; ofs addrL addrH
    bra prt01
ti: ; special treatment for I tetriminos
    clrf T
    lslf angle,W
    addwf T,F
    movlw low(I0) 
    push     ; ofs addrL
    movlw high(I0)
    push     ; ofs addrL addrH
prt01:    
    call get_flash_word  ; --
    banksel tx
    movfw tx
    push    ; -- x
    movfw ty
    push    ; -- x y 
    lit 0   ; -- x y f
    call print_row ; x y f -- x y+1 f
    call print_row
    call print_row
    banksel PMDAT
    movlw 0x30
    andwf PMDATH,W
    skpnz
    call print_row
    movfw T
    drop_n 3
    push  ; -- f
    return
    
;print a text line store in flash
; text terminated by 0xff
; input: 
;  x, y left coordinates
;  idx  index of message    
print_label: ; ( x y idx -- )
    lit 0    ; x y idx i
prt_lbl_loop:
    dup      ; x y idx i i
    pick 2   ; WREG=idx
    call labels ; x y idx i i -- x y idx i  
    btfsc WREG,7
    bra prt_lbl_done
    push ; x y idx i c
    pick 4
    push    ; x y idx i c x 
    pick 4
    push    ; x y idx i c x y
    rot	    ; x y idx i x y c
    call print_char ; -- x y idx i 
    incf T
    pick 3    ; x
    addlw 4   ; x+=4
    insert 3  
    bra prt_lbl_loop
prt_lbl_done:
    drop_n 4
    return

;print an integer
;from right to left ( least first)
; input:    
;   x,y  coordinate right end
;   number to print in acca    
;   accbL contain digit after div10 call    
print_int: ; ( x y -- )
    movfw accaH
    iorwf accaL,W
    skpnz
    bra print_last
    over
    over    ; x y x y
    call div10
    movfw accbL ; remainder of division
    push   ; x y x y d
    call print_char ; -- x y 
    pick 1
    push
    movlw 4
    subwf T
    pop
    insert 1
    bra print_int
print_last:
    pick 1
    xorlw 43
    skpz
    bra no_zero
    push
    bra print_char
no_zero:
    drop_n 2
    return
    
; draw horizontal line ( length  y x -- )
; inputs:
;   length of line
;   {x,y} left coordinates       
hline:
    pick 2
    skpnz
    bra hline_done
    decf WREG
    insert 2
    dup	    ; len y x x
    pick 2
    push    ; len y x x y
    lit XOR_OP
    call bitop
    drop
    incf T  ; len y x+1
    bra hline
hline_done:    
    drop_n 3
    return

;draw vertical line ( length x y -- )
; inputs:
;   length of line
;   {x,y} top coordinate
vline:  ; len y x
    pick 2 
    skpnz
    bra vline_done
    decf WREG
    insert 2
    pick 1	
    push    ; len x y x
    pick 1
    push    ; len x y x y
    lit XOR_OP
    call bitop
    drop
    incf T
    bra vline
vline_done:
    drop_n 3
    return
    
WELL_WIDTH equ 10
WELL_DEPTH equ 22 
game_init:
;clear screen    
    lit 0  
    call fill_buffer
; draw game well    
    lit WELL_DEPTH
    lit 0
    lit 0
     call vline
    lit WELL_DEPTH
    lit WELL_WIDTH+1
    lit 0
    call vline
    lit WELL_WIDTH+2
    lit WELL_DEPTH
    lit 0
    call hline
; print "SCORE" label    
    lit WELL_WIDTH+3 ; x
    lit 0	     ; y
    lit LBL_SCORE	  
    call print_label
; print "LINES" label    
    lit WELL_WIDTH+3 ; x
    lit 12	     ; y
    lit LBL_LINES
    call print_label
; variables initialization
    banksel 0
    movlw 4
    movwf tx
    clrf ty
    clrf tetrim
    clrf angle
    clrf scoreL
    clrf scoreH
    clrf dropped
    clrf buttons
    bsf flags,F_GSTOP
    return

ROW_EMPTY equ 0     
ROW_FULL equ 10 ; 10 bits in row
; check the state of well row
; input:
;   s number of bits set initialized at 0 by caller    
;   r row number: 0 top, 22 bottom  
; output:    
;   s={0:10}  number of bits set
query_row: ; ( s=0 r -- s=0:10 )
    lit 10 ; s r x   ; check x from  10 to 1
qr00:
    dup   ; s r x x
    pick 2 
    push  ; s r x x y
    call get_pixel ; -- s r x n
    skpnz   ; Z modified by get_pixel
    bra qr01
    inc_n 3
qr01:    
    drop     ; -- s r x 
    decfsz T
    bra qr00
    drop_n 2 ; -- s
    return
 
; generate a new tetriminos
; input:
;   none
; output:
;   Z flag set if collision
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
    return
 
; wait player start signal
; button B pressed    
wait_start:
    clrw
    call korobeiniki
    push
    lit 0
koro:
    incf T
    movfw T
    call korobeiniki
    push
    inc_n 1
    pick 1
    call korobeiniki
    push
    call tone
    wait_sound
    call read_pad
    btfsc buttons, BTN_A
    bra wait_end
    dec_n 1
    skpz
    bra koro
    pause 60
    bra wait_start
wait_end:    
    drop_n 2
    return
    
;;;;;;;;;;;;;;;;;;;;;;;;;
;   game logic
;;;;;;;;;;;;;;;;;;;;;;;;;    
tetris:
; print score
    banksel scoreL
    movfw scoreL
    movwf accaL
    movfw scoreH
    movwf accaH
    lit 43
    lit 6
    call print_int
;print dropped line
    banksel dropped
    movfw dropped
    movwf accaL
    clrf accaH
    lit 43
    lit 18
    call print_int
;print start prompt
    lit 0
    lit 24
    lit LBL_PRESS
    call print_label
; wait button A press
; to start game    
    call wait_start 
    bcf flags, F_GSTOP
; delete prompt
    lit 0
    lit 24
    lit LBL_PRESS
    call print_label
; game start
game_loop:
; generate new tetriminos
; if collision at this stage
; game is over
    call new_tminos ; Z return collision status
fall_loop:
    call print_tetrim
    pop
    skpnz
    bra game_over
    pause 10
    call print_tetrim
    pop
; read pad
    call read_pad
    banksel GAME_VAR
    movfw buttons
    case 1<<BTN_A, hard_drop
    case 1<<BTN_B, soft_drop
    case 1<<BTN_UP, rot_right
    case 1<<BTN_DN, rot_left
    case 1<<BTN_RT, move_right
    case 1<<BTN_LT, move_left
    bra new_position
hard_drop:
    bra new_position
soft_drop:
    bra new_position
rot_left:
    decf angle,F
    movlw 3
    andwf angle,F
    bra new_position
rot_right:
    incf angle,F
    movlw 3
    andwf angle,F
    bra new_position
move_left:
    movlw 1
    xorwf tx,W
    skpz
    decf tx,F
    bra new_position
move_right:
    movlw 10
    xorwf tx,W
    skpz
    incf tx,F
; apply move and rotation
new_position:    
; check full row and clean
; adjust score 
    bra fall_loop
game_over:
    call print_tetrim
    pop
    
    return
    
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
; audio pwm initialization
    banksel PWM2PH
    clrf PWM2PHL
    clrf PWM2PHH
    clrf PWM2OFL
    clrf PWM2OFH
    bsf AUDIO_PWMCON,OE
    banksel APFCON
    bsf APFCON,P2SEL
   
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

    
main:
    call game_init
    call tetris
    bra main
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;   data tables
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; **************** encoding ************************************************
; enhanced PIC instructions are coded on 14 bits
; these 14 bits can be read as data using flash memory
; writing/reading mechanism, see datasheet chapter 10 for detail.
; the 'dw' assembler directive encode data in those 14 bits contrary to 'dt'
; directive which encode a single byte as a RETLW instruction.
; So we get a 50% compression by encoding 3 nibbles/flash word. 
; the encoding used in the following tables is as follow:    
; bits 13:12  number of nibbles where 0 means there is more than 3 nibbles
; in this data chunk so 1 or more other words follow this one.    
; other 12 bits are sliced in 4 bits nibbles.
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
    dw 0x3EAE,0x2AA0 ; A  code 10
    dw 0x3688,0x2860 ; C  code 11
    dw 0x3E8E,0x28E0 ; E  code 12
    dw 0x3E44,0x24E0 ; I  code 13
    dw 0x3888,0x28E0 ; L  code 14
    dw 0x38CA,0x2AA0 ; N  code 15
    dw 0x34AA,0x2A40 ; O  code 16
    dw 0x3EAE,0x2880 ; P  code 17
    dw 0x38EA,0x2880 ; R  code 18
    dw 0x3684,0x22C0 ; S  code 19
    dw 0x3040,0x2400 ; :  code 20
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
I0: dw 0x0444,0x1400 ; I R0  
    dw 0x300F,0x1000 ; I R1  second word is filling for alignment
    dw 0x0222,0x1200 ; I R2
    dw 0x10F0,0x1000 ; I R3  

LBL_SCORE equ 0
LBL_LINES equ 1
LBL_PRESS equ 2
 
labels:
    brw
    bra txt_score
    bra txt_lines
    bra txt_press
    
txt_score: ; "SCORE"
    pop
    brw
    dt 19,11,16,18,12,20,255
    
txt_lines: ; "LINES"
    pop
    brw
    dt 14,13,15,12,19,20,255

txt_press: ; "PRESS A"
    pop
    brw
    dt 17,18,12,19,19,21,10,255
    

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
    
korobeiniki: ;folklore russe
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
    
    
    end

    