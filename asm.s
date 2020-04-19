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
   .def pushReg
   .def popReg
   .def getPsp
   .def getSvcNo
   .def getR0
   .def setLr


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
			   MRS    R2, CONTROL
			   MOV R1, #2
			   ORR R1, R2, R1
               MSR    CONTROL, R1
               MSR    PSP, R0                ;
               ;ISB
               BX     LR                     ; return from subroutine
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
	           MRS R0, PSP
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
getSvcNo:
			   MRS R0, PSP
			   ADD R0, R0, #24
			   LDR R0, [R0]
			   SUB R0, #2
			   LDR R0, [R0]
			   AND R0, #15
			   BX LR
getR0:
			   MRS R0, PSP
;			   LDR R0, [R0]
			   BX LR
setLr:
			  MOV R0, #0xFD
			  MOV R1, #0XFF
			  LSL R2, R1, #8
			  LSL R3, R1, #16
			  LSL R4, R1, #24
			  ORR R0, R0, R2
			  ORR R0, R0, R3
			  ORR R0, R0, R4
			  MOV LR, R0
			  BX LR
.endm
