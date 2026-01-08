; ******************************************************************************
; * © 2008 Microchip Technology Inc.
; *
; SOFTWARE LICENSE AGREEMENT:
; Microchip Technology Incorporated ("Microchip") retains all ownership and 
; intellectual property rights in the code accompanying this message and in all 
; derivatives hereto.  You may use this code, and any derivatives created by 
; any person or entity by or on your behalf, exclusively with Microchip's
; proprietary products.  Your acceptance and/or use of this code constitutes 
; agreement to the terms and conditions of this notice.
;
; CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
; WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
; TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
; PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S 
; PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
;
; YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, 
; WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF 
; STATUTORY DUTY), STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR 
; ANY INDIRECT, SPECIAL, PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, 
; DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE,
; HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE
; DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW, MICROCHIP'S 
; TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS CODE, SHALL NOT 
; EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO HAVE THIS CODE
; DEVELOPED.
;
; You agree that you are solely responsible for testing the code and 
; determining its suitability.  Microchip has no obligation to modify, test, 
; certify, or support the code.
;
; *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        ; Global inclusions.
        .nolist
        ; none...
        .list

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Some constants:
        .equ    kSof,2                          ; sizeof (fractional), or
                                                ; sizeof (fractional*) if
                                                ; addressing data memory
        .equ    kSinPiQ,0x5A82                  ; sin (pi/4)
        .equ    kInvSqrt2,kSinPiQ               ; 1/sqrt(2) = sin (pi/4)

;............................................................................

        ; Declare special function registers as of DS70030C page 4-3,4
        ; since IAR does not recognize the symbols CORCON, MODCON, ...
        .equ    PSVPAG,0x0034
        .equ    CORCON,0x0044
        .equ    MODCON,0x0046
        .equ    XBREV,0x0050
        .equ    XMODSRT,0x0048
        .equ    XMODEND,0x004A
        .equ    YMODSRT,0x004C
        .equ    YMODEND,0x004E

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

        ; Operational modes for fractional computation:
        .equ    SATA_ON,1<<7                    ; AccA sat. enabled
        .equ    SATB_ON,1<<6                    ; AccB sat. enabled
        .equ    SATDW_ON,1<<5                   ; data write sat. enabled
        .equ    ACCSAT_ON,1<<4                  ; Accs sat. 9.31 enabled
        .equ    FRACT_SET,SATA_ON|SATB_ON|SATDW_ON|ACCSAT_ON    ; set mask

        .equ    RND_OFF,~(1<<1)                 ; convergent rnd. enabled
        .equ    IF_OFF,~(1<<0)                  ; multiply fractional enabled
        .equ    FRACT_RESET,RND_OFF&IF_OFF      ; reset mask

        .equ    FRACT_MODE,FRACT_SET&FRACT_RESET        ; fractional mask

        .ifdef  TOOLVERS                        ; [
        .equ    FRACT_MODE,FRACT_MODE&0x3ff     ; get rid of leading (sign)
                                                ; bits so that the mask can
                                                ; be used as a Slit10
        .endif  ; ]

        ; Operational modes for Program Space Visibility.
        .equ    PSV_ON,1<<2                     ; program space visible in
                                                ; data space

;............................................................................

        ; Prepare CORCON for fractional computation.
        .macro  fractsetup      wx              ; [
        mov     #FRACT_MODE,\wx
        mov     \wx,CORCON
        .endm   ; ]

        ; Prepare CORCON for Program Space Visibility access.
        .macro  psvaccess       wx              ; [
        mov     CORCON,\wx
        ior     #PSV_ON,\wx
        mov     \wx,CORCON
        .endm   ; ]

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; FIR filter structure access.
        .equ    COEFFS_IN_DATA,0xFF00   ; indicates that coefficients are in
                                        ; (X) data memory.

        .equ    oNumCoeffs,0            ; FIRFilter->numCoeffs : (int)
        .equ    oCoeffsBase,(oNumCoeffs+kSof)
                                        ; FIRFilter->coeffsBase : (fractional*)
        .equ    oCoeffsEnd,(oCoeffsBase+kSof)
                                        ; FIRFilter->coeffsEnd : (fractional*)
        .equ    oCoeffsPage,(oCoeffsEnd+kSof)
                                        ; FIRFilter->coeffsPage : (int)
        .equ    oDelayBase,(oCoeffsPage+kSof)
                                        ; FIRFilter->delayBase : (fractional*)
        .equ    oDelayEnd,(oDelayBase+kSof)
                                        ; FIRFilter->delayEnd : (fractional*)
        .equ    oDelay,(oDelayEnd+kSof)
                                        ; FIRFilter->delay : (fractional*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; IIR Lattice filter structure access.
        .equ    oOrder,0                ; filter->order : (int)
        .equ    oKappaVals,(oOrder+kSof); filter->kappaVals : (fractional*)
        .equ    oGammaVals,(oKappaVals+kSof); filter->gammaVals : (fractional*)
        .equ    oLattCoeffsPage,(oGammaVals+kSof); filter->coeffsPage : (int)
        .equ    oLattDelay,(oLattCoeffsPage+kSof); filter->delay : (fractional*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; OEF
