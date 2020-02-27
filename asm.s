; Stop Go C/ASM Mix Example
; Jason Losh

;-----------------------------------------------------------------------------
; Hardware Target
;-----------------------------------------------------------------------------

; Target Platform: EK-TM4C123GXL Evaluation Board
; Target uC:       TM4C123GH6PM
; System Clock:    40 MHz

; Hardware configuration:
; Red LED:
;   PF1 drives an NPN transistor that powers the red LED
; Green LED:
;   PF3 drives an NPN transistor that powers the green LED
; Pushbutton:
;   SW1 pulls pin PF4 low (internal pull-up is used)

;-----------------------------------------------------------------------------
; Device includes, defines, and assembler directives
;-----------------------------------------------------------------------------

   .def setSp
   .def setPsp
   .def turnPspOn
   .def callSvc
   .def pushReg
   .def popReg
   .def getPsp


;-----------------------------------------------------------------------------
; Register values and large immediate values
;-----------------------------------------------------------------------------

.thumb
.const

;-----------------------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------

.text

; Blocking function that returns only when SW1 is pressed
setSp:
               MOV    R13, R0                ;
               BX     LR                     ; return from subroutine
setPsp:
               MSR    PSP, R0                ;
               BX     LR                     ; return from subroutine
turnPspOn:
			   MRS    R0, CONTROL
			   MOV R1, #2
			   ORR R1, R0, R1
               MSR    CONTROL, R1
               MRS SP, MSP
               ISB	         ;
               BX     LR                     ; return from subroutine
callSvc:
	           SVC #100
	           BX LR
pushReg:
	           MRS R0, PSP
	           STR R4, [R0]
	           SUB R0, R0, #4
	           STR R5, [R0]
	           SUB R0, R0, #4
	           STR R6, [R0]
	           SUB R0, R0, #4
	           STR R7, [R0]
	           SUB R0, R0, #4
	           STR R8, [R0]
	           SUB R0, R0, #4
	           STR R9, [R0]
	           SUB R0, R0, #4
	           STR R10, [R0]
	           SUB R0, R0, #4
	           STR R11, [R0]
	           MSR PSP, R0
	           BX LR
popReg:
	           MSR PSP,R0
	           LDR R11, [R0]
	           ADD R0, R0, #4
	           LDR R10, [R0]
	           ADD R0, R0, #4
	           LDR R9, [R0]
	           ADD R0, R0, #4
	           LDR R8, [R0]
	           ADD R0, R0, #4
	           LDR R7, [R0]
	           ADD R0, R0, #4
	           LDR R6, [R0]
	           ADD R0, R0, #4
	           LDR R5, [R0]
	           ADD R0, R0, #4
	           LDR R4, [R0]
	           MSR PSP, R0
	           BX LR
getPsp:
	           MRS R0, PSP
	           BX LR
.endm
