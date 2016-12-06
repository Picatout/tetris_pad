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
VERT_DLY equ 40
FIRST_VIDEO equ VERT_DLY+20  ; first video output line
LAST_VIDEO equ FIRST_VIDEO+4*YSIZE-1  ; last video output line
PIXEL_TXREG equ TXREG
TX_PIR equ PIR1
HORZ_DLY equ 16*5  ; there is 5 TCY/µsec
 
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
HORZ_PULSE equ 4700/TC ; 4.7µsec 
LAST_LINE equ 262

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
; read threshold level for each button 
VDD equ 3
B_THR equ VDD*5/6-4
A_THR equ VDD*4/5-4
DN_THR equ VDD*3/4-4 
LT_THR equ VDD*2/3-4
UP_THR equ VDD*1/2-4
RT_THR equ 0
 
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
    
tcyDelay macro n ; delay in TCY cycles  5TCY per µsec.  
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
    start_timer
    wait_timer
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
    udata 0x20
; parameter stack area    
_stack res 0x20
; tetris game state
tetrim res 1 ; active tretriminos 
angle res 1 ; tetriminos rotatation angle in multiple of 90degr. {0:3}
tx  res 1 ; x coordinate
ty  res 1 ; y coordinate
scoreL res 1 ; game score  16 bits
scoreH res 1 
dropped res 1 ; dropped lines

 
    udata_shr
;scan lines counter 16 bits
lcountL res 1
lcountH res 1
; boolean flags 
flags res 1
; game timer
gtimer res 1 
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
    btfss flags, F_VSYNC
    bra isr01
    movfw lcountL
    xorlw 3
    skpz
    bra isr_exit
vsync_end:
    banksel SYNC_PWMDCL
    movlw HORZ_PULSE&0xff
    movwf SYNC_PWMDCL
    movlw HORZ_PULSE>>8
    movwf SYNC_PWMDCH
    bsf SYNC_PWMLDCON,7
    bcf flags, F_VSYNC
    bra isr_exit
isr01:    
    movfw lcountL
    xorwf lcountH,W
    skpz
    bra isr02
vsync_start:
    banksel SYNC_PWMDCL
    movlw (HORZ_PERIOD-HORZ_PULSE)&0xff
    movwf SYNC_PWMDCL
    movlw (HORZ_PERIOD-HORZ_PULSE)>>8
    movwf SYNC_PWMDCH
    bsf SYNC_PWMLDCON,7
    bsf flags, F_VSYNC
    bra isr_exit
isr02:
    btfss lcountH,0
    bra lt_256
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
    xorwf flags
    bra isr_exit
; round robin task scheduler
; each task execute once every 1/60th sec.
; condition:    
;   Task must complete inside
;   horizontal period.
tasks:
    movfw lcountL
    case 1, task1
    case 2, task2
    bra isr_exit
task1:   
; decrement game timer    
    movf gtimer,F ; 42Tcy
    skpnz
    bra isr_exit
    decf gtimer,F ; 44Tcy
    skpnz
    bcf flags, F_GTMR
    bra isr_exit ; 46Tcy=5.75µSec
task2:
; rotate lfsr
    lsrf randH
    rrf randL
    skpnc
    movlw LFSR_TAPS
    xorwf randH
    bra isr_exit ; 53Tcy
lt_256:
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
isr_exit:
    incf lcountL
    skpnz
    incf lcountH
    retfie

; WREG*6  because BPL=6
; WREG*6=WREG*4+WREG*2    
mult6: 
    movwf accaL
    lslf accaL    ; accaL=2*WREG
    lslf accaL,W  ; WREG=4*WREG
    addwf accaL,W ; WREG=6*WREG
    return

;read game pad
; store value in
; buttons    
read_pad:
    banksel ADCON0
    bsf ADCON0,ADON
    bsf ADCON0,GO
    btfsc ADCON0,NDONE
    bra $-1
    bcf ADCON0,ADON
    movlw B_THR
    subwf ADRESH,W
    skpc
    
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
fill_buffer: 
    reserve 1
    push
    disable_video
    movlw high(VIDEO_BUFFER)
    movwf FSR0H
    movlw low(VIDEO_BUFFER)
    movwf FSR0L
    movlw BUFFER_SIZE
    insert 1
    pop
fill_loop:    
    movwi FSR0++
    decfsz T
    bra fill_loop
    drop
    enable_video
    return

;set video pointer at 
; byte that contain pixel {x,y}
; output:
;   WREG = pixel offset, 0 left, 7 right  
;   Z=0 if offset==0    
set_video_ptr:  ; ( x y -- )
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
    pop            
    andlw 7
    return

;inverse le pixel    
; inputs:
;   {x,y} coordinates
; output:
;   WREG=collision flag    
xor_pixel: ; ( x y -- )
    disable_video
    pick 1
    push
    pick 1
    push	; x y x y
    movfw FSR0L
    insert 3
    movfw FSR0H
    insert 2	; FSR0L FSR0H x y 
    call set_video_ptr  ;FSR0L FSR0H 
    push	; FSR0L FSR0H bit
    movlw 0x1
    skpnz
    bra xorp01
xorp00:
    lslf WREG
    decfsz T
    bra xorp00
xorp01:
    xorwf INDF0
    andwf INDF0,W ; on screen bit value 0|1 for collision detection
    movwf T
    pick 1  
    movwf FSR0H
    pick 2
    movwf FSR0L
    pop  ; collistion flag 0 -> no collision
    drop_n 2
    push ; return collision flag
    enable_video
    return

; draw row of pixels
; draw up to 8 pixels.
; stop when r==0    
; input:
;   {x,y} left coordinates
;   r pixels to draw
; output:
;   f=collision flag, return modified value   
xor_row: ; ( f r x y -- f )
    pick 2 ; check if r==0
    skpnz
    bra xor_row_done ; r==0 done
    lslf WREG
    insert 2  ; save shifted r
    skpc
    bra xor_row02 ; bit==0 no draw
    over ; f r x y x 
    over ; f r x y x y
    call xor_pixel ; f r x y fn
    pick 4 ; pick flag
    iorwf T
    pop	    
    insert 3 ; store modified flag
xor_row02:
    inc_n 1  ; x+=1
    bra xor_row
xor_row_done: ; f r x y
    drop_n 3  ; only keep f
    return

; read flash memory word
; input:
;   lo is low byte of address
;   hi is high byte of address
;   ofs offset in table (limited to 255 )    
; output:
;   PMDATH: PMDAL 
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
    return

; increament pointer 
; and read next flash word    
next_flash_word:
    incf PMADRL
    skpnz
    incf PMADRH
    bra read_flash
    
;get row from PMDAT    
get_row: ; ( n -- row )    
    movfw T
    skpnz
    bra row0
    decfsz WREG
    bra row2
row1:    
    movfw PMDATL
    bra nibble
row2:
    swapf PMDATL,W
    bra nibble
row0:    
    swapf PMDATH,W
nibble:
    andlw 0xf0
    movwf T
    return

; print 4 pixels row
; inputs:
;   x,y left coordinates
;   f  collision flag
;   r  row index {0-2}
; advance y coordinate for next row    
print_row: ; ( x y f r -- x y f )     
    call get_row ; x y f n 
    pick 3 
    push    ; x y f n x
    pick 3
    push      ; x y f n x y
    call xor_row ; x y f
    inc_n 1
    return
    
; draw character 
; input:
;   x,y left/top coordinates
;   c   character to print    
print_char: ; ( x y c -- )
    lslf T ; 2 words per digit
    lit low(digits)
    lit high(digits)
    call get_flash_word  ; x y
    lit 0   ; x y f=0
    lit 0
    call print_row
    lit 1
    call print_row
    lit 2
    call print_row   
    call next_flash_word
    lit 0
    call print_row 
    lit 1 
    call print_row ; x y f r -- x y f
    drop_n 3
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
    bra prt_tbl_done
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
prt_tbl_done:
    drop_n 4
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
    call xor_pixel
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
    call xor_pixel
    drop
    incf T
    bra vline
vline_done:
    drop_n 3
    return
    
WELL_WIDTH equ 10
WELL_DEPTH equ 22 
game_init:
    lit 0   ; lit 0 to clear screen black
    call fill_buffer ; clear screen white
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
    lit WELL_WIDTH+3 ; x
    lit 0	     ; y
    lit LBL_SCORE	     ; message index
    call print_label
    lit WELL_WIDTH+3 ; x
    lit 12	     ; y
    lit LBL_LINES       ; message index
    call print_label
    lit WELL_WIDTH+3 ; x
    lit 6	     ; y
    lit LBL_ZEROS       ; message index
    call print_label
    lit WELL_WIDTH+3 ; x
    lit 18	     ; y
    lit LBL_ZEROS       ; message index
    call print_label
    
; variables initialization
    banksel 0
    movlw 2
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
    
tetris:
    
    return
    
init:
    pstack_init
; ADC configuration
    banksel ADCON0
    movlw (PAD_CHS<<CHS0)
    movwf ADCON0
    movlw (2<<ADCS0)
    movwf ADCON1
;;;;;;;;;;;;;;;;;;;;;    
    banksel TRISA
    bcf TRISA, SYNC_OUT
    bcf TRISA, VIDEO_OUT
    banksel VIDEO_LAT
    bcf VIDEO_LAT,VIDEO_OUT
;configure EUSART in sychronsous mode
;to use as pixel serializer
    banksel SPBRG
    movlw 1
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
    bsf SYNC_PWMLDCON,7
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
    bra $
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
    dw 0x3688,0x2860 ; C  code 10
    dw 0x3E8E,0x28E0 ; E  code 11
    dw 0x3E44,0x24E0 ; I  code 12
    dw 0x3888,0x28E0 ; L  code 13
    dw 0x38CA,0x2AA0 ; N  code 14
    dw 0x34AA,0x2A40 ; O  code 15
    dw 0x38EA,0x2880 ; R  code 16
    dw 0x368C,0x22C0 ; S  code 17
    dw 0x3040,0x2400 ; :  code 18
; annoying! I tetriminos need a special treatment 
; because vertical I need 2 words for encoding.   
I0: dw 0x0222,0x1200 ; I R0  
    dw 0x1F00,0x1000 ; I R1  second word is filling for alignment
    dw 0x0222,0x1200 ; I R2
    dw 0x1F00,0x1000 ; I R3  

; comments notation: Rn clockwise rotation n*90 degr.    
; note that vertical I as 4 rows so it needs 2 words    
tetrominos: 
    dw 0x3445 ; L R0
    dw 0x2E80 ; L R1
    dw 0x3622 ; L R2
    dw 0x22E0 ; L R3
    dw 0x3226 ; J R0
    dw 0x28E0 ; J R1
    dw 0x3644 ; J R2
    dw 0x2E20 ; J R3
    dw 0x2660 ; O R0 
    dw 0x2660 ; O R1
    dw 0x2660 ; O R2
    dw 0x2660 ; O R3
    dw 0x2360 ; S R0
    dw 0x3462 ; S R1
    dw 0x2360 ; S R2
    dw 0x3462 ; S R3
    dw 0x2E40 ; T R0
    dw 0x34c4 ; T R1
    dw 0x24E0 ; T R2
    dw 0x3464 ; T R3
    dw 0x2C60 ; Z R0
    dw 0x3264 ; Z R1
    dw 0x2C60 ; Z R3
    dw 0x3264 ; Z R4

LBL_SCORE equ 0
LBL_LINES equ 1
LBL_ZEROS equ 2
 
labels:
    brw
    bra txt_score
    bra txt_lines
    bra txt_zeros
    
txt_score:
    pop
    brw
    dt 17,10,15,16,11,18,255
    
txt_lines:
    pop
    brw
    dt 13,12,14,11,17,18,255

txt_zeros:
    pop
    brw
    dt 0,0,0,0,0,0,0,0,255
    
    end


