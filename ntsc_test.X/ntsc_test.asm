    include p12f1572.inc
    
    radix dec
    
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
F_VSYNC equ 0
F_EVEN equ 1
F_MUTEX equ 2
 
disable_video macro
    bsf flags, F_MUTEX
    endm
    
enable_video macro
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
    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; software stack for arguments passing
; SP is Stack Pointer
; T is Top Of Stack
#define SP FSR1
#define T INDF1

 
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
    addwf T
   endm
   
sub macro  ; substract WREG from T result on T
   subwf T
   endm

inc_n macro n ; increament nth element of stack
    moviw -n[SP]
    incf WREG
    movwi -n[SP]
    endm
    
dec_n macro n ; decreament nth element of stack
    moviw -n[SP]
    decf WREG
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
    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    
    udata_shr
;scan lines counter    
lcountL res 1
lcountH res 1
; boolean flags 
flags res 1
rcount res 1
temp res 1
accaL res 1
accaH res 1
 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 
    org 0
    banksel ANSELA
    clrf ANSELA
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
    bra isr_exit
    clrf lcountL
    clrf lcountH
    movlw 1<<F_EVEN
    xorwf flags
    bra isr_exit
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
    movwf temp
    lslf temp
    lslf temp,W
    addwf temp,W
    return
    
; serialise video pixels to scan line
; due to timing constrain use specialized 
; division and multiplication    
video_serialize: 
;    movfw FSR0L
;    push
;    movfw FSR0H
;    push
;    lit 0 ; ( x )
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
;    pop
;    movwf FSR0H
;    pop
;    movwf FSR0L
    return

; fill_buffer
; fill screen buffer with WREG value    
fill_buffer: 
    movwf temp
    disable_video
    movlw VIDEO_BUFFER>>8
    movwf FSR0H
    movlw VIDEO_BUFFER&0xff
    movwf FSR0L
    movlw BUFFER_SIZE
    movwf rcount
    movfw temp
fill_loop:    
    movwi FSR0++
    decfsz rcount
    bra fill_loop
    enable_video
    return

;set video pointer at 
; byte that contain pixel {x,y}
; output:
;   WREG = pixel offset, 0 left, 7 right  
;   Z=0 if offset==0    
set_video_ptr:  ; ( x y -- )
    movlw VIDEO_BUFFER>>8
    movwf FSR0H
    movlw VIDEO_BUFFER&0xff
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

    
xor_pixel: ; ( x y -- ) inverse le pixel
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
    drop	; FSR0L FSR0H
    xorwf INDF0 
    pop		; FSR0L
    movwf FSR0H
    pop		; vide
    movwf FSR0L
    enable_video
    return

; draw horizontal line ( length  y x -- )
; inputs:
;   length of line
;   {x,y} left,top coordinates       
#define xcoord accaL    
hline:
    pick 2
    push
    zbranch hline_done
    dup
    pick 2
    push
    call xor_pixel
    incf T
    dec_n 2
    bra hline
hline_done:    
    drop_n 3
    return
    
; draw box ( height width top left )
box:
    pick 2
    push
    pick 2
    push
    pick 2
    push
    call hline
    inc_n 1
    dec_n 3 ; use height argument as line counter
    skpz
    bra box
    drop_n 4 ; clean stack frame
    return
    

WELL_WIDTH equ 10
WELL_DEPTH equ 22 
game_init:
    lit 0   ; lit 0 to clear screen black
    call fill_buffer ; clear screen white
    lit WELL_DEPTH+1
    lit WELL_WIDTH+2
    lit 0
    lit 0
    call box
    ; draw game well
    lit WELL_DEPTH
    lit WELL_WIDTH
    lit 0
    lit 1
    call box
    ; initialise variables
    return
    
tetris:
    
    return
    
init:
    pstack_init
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
main:
    call game_init
    call tetris
    bra $
    bra main
    
    end


