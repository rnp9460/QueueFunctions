            TTL ;Lab 10
;****************************************************************
;Name:  Rhythm Patel
;Date:  11/11/2020
;Class:  CMPE-250
;Section:  Thursday 2:00, Section 4
;---------------------------------------------------------------
;Keil Template for KL05
;R. W. Melton
;September 13, 2020
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL05Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
;****************************************************************
;EQUates
;OutPut Equates
CR				  EQU  0X0D
LF				  EQU  0X0A
NULL			  EQU  0X00
	
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
	
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
	
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << PIT_PRI_POS)
	
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)

NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
	

IT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK

PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
	
PORT_PCR_SET_PTB2_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTB1_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
								
SIM_SOPT2_UART0SRC_MCGFLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)

SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)

UART0_BDH_9600  EQU  0x01

UART0_BDL_9600  EQU  0x38

UART0_C1_8N1  EQU  0x00

UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)

UART0_C3_NO_TXINV  EQU  0x00

UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16

UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00

UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
						
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)

PIT_MCR_EN_FRZ	EQU		PIT_MCR_FRZ_MASK
	
PIT_LDVAL_10ms	EQU		0x3A97F;239,999 to hex

PIT_IRQ_PRI	EQU	0

IN_PTR      EQU   0
OUT_PTR     EQU   4
BUF_STRT    EQU   8
BUF_PAST    EQU   12
BUF_SIZE    EQU   16
NUM_ENQD    EQU   17
; Queue structure sizes
Q_BUF_SZ    EQU   4   ;Queue contents
Q_REC_SZ    EQU   18  ;Queue management record
;--------------------------------------------------------------
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
            IMPORT  Startup
			IMPORT 	Init_UART0_IRQ
			IMPORT 	UART0_ISR
			IMPORT 	GetChar
			EXPORT	Enqueue
			EXPORT	Dequeue
			EXPORT	InitQueue
			

Reset_Handler  PROC  {}
min
;---------------------------------------------------------------
;Mask interrupts
            CPSID   I
;KL05 system startup with 48-MHz system clock
            BL      Startup
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
;Main program to take a character and output it using GetChar and PutChar using a loop
			BL 		Init_UART0_IRQ	 ;Initialise UART IRQ I/O
			
			LDR		R0,=RunStopWatch ;ssetting the variable to 0
			MOVS	R1,#0
			STR		R1,[R0,#0]
			
			LDR		R0,=Count ;setting the variable to 0
			MOVS	R1,#0
			STR		R1,[R0,#0]
			
			BL		Init_PIT_IRQ
			CPSIE	I
PROMPT_OUT ;Label to print the prompt for user
			MOVS	R0,#CR
			BL 		PutChar
			MOVS	R0,#LF
			BL		PutChar
			LDR		R0,=Prompt
			MOVS	R1,#(Prompt_past-Prompt)
			BL		PutStringSB

			BL		GetChar ;wait for the user input
			
;-----------------------------------			
C_CHECK
			CMP		R0,#'C' 
			BEQ		C_FUNC
			CMP		R0,#'c'
			BNE		D_CHECK ;if the entered letter is C, then it goes to the C_FUNC 
C_FUNC
			MOVS	R0,R7

			MOVS	R0,#'>'
			BL		PutChar
			MOVS	R0,R7
			BL		PutChar
			MOVS	R0,#CR
			BL		PutChar
			MOVS	R0,#LF
			BL		PutChar
			MOVS	R0,#'>'
			BL		PutChar
			
			LDR		R0,=Count ;set the count variable to 0 so that the count is cleared
			MOVS	R1,#0
			STR		R1,[R0,#0]
			
			B		PROMPT_OUT
;------------------------------------
D_CHECK
			CMP		R0,#'D'
			BEQ		D_FUNC
			CMP		R0,#'d'
			BNE		H_CHECK ;if the entered letter is D or d, then it goes to the D_FUNC
D_FUNC
			MOVS	R0,R7
			MOVS	R0,#'>'
			BL		PutChar
			MOVS	R0,#CR
			BL		PutChar
			MOVS	R0,#LF
			BL		PutChar
			MOVS	R0,#'>'
			BL		PutChar
			MOVS	R0,R7
			BL 		PutChar
			MOVS	R0,#':'
			BL		PutChar
			
			LDR		R2,=Count ;The count variable is outputted
			LDR		R0,[R2,#0]
			BL		PutStringSB
			
			LDR		R0,=time_factor
			MOVS	R1,#(time_factor_past - time_factor)
			BL		PutStringSB
			
			B		PROMPT_OUT
;------------------------------------
H_CHECK
			CMP		R0,#'H'
			BEQ		H_FUNC
			CMP		R0,#'h'
			BNE		P_CHECK ; if the entered letter is H or h, then it goes to the H_FUNC
H_FUNC
			MOVS	R0,R7
			MOVS	R0,#'>'
			BL		PutChar
			MOVS	R0,#CR
			BL		PutChar
			MOVS	R0,#LF
			BL		PutChar
			MOVS	R0,#'>'
			BL		PutChar
			MOVS	R0,R7
			BL 		PutChar
			MOVS	R0,#':'
			BL		PutChar
			
			LDR		R0,=Help ;help prompt is printed
			MOVS	R1,#(Help_past - Help)
			BL		PutStringSB
			
			B		PROMPT_OUT
			
;------------------------------------
P_CHECK
			CMP		R0,#'P'
			BEQ		P_FUNC
			CMP		R0,#'p'
			BNE		T_CHECK ;If the entered letter is P or p, then it goes to the P_FUNC
P_FUNC
			MOVS	R0,R7
			MOVS	R0,#'>'
			BL		PutChar
			MOVS	R0,R7
			BL 		PutChar
			MOVS	R0,#CR
			BL		PutChar
			MOVS	R0,#LF
			BL		PutChar
			MOVS	R0,#'>'
			BL		PutChar
			
			LDR		R0,=RunStopWatch
			MOVS	R1,#0
			STR		R1,[R0,#0]
			
			B		PROMPT_OUT
			
;------------------------------------
T_CHECK
			CMP		R0,#'T'
			BEQ		T_FUNC
			CMP		R0,#'t' 
			BNE		PROMPT_OUT ;if the entered letter is T or t, then it goes to the T_FUNC
T_FUNC
			LDR		R0,=RunStopWatch
			MOVS	R1,#1
			STR		R1,[R0,#0]

			B		PROMPT_OUT
			
;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP
;>>>>> begin subroutine code <<<<<
Init_PIT_IRQ PROC {R0-R14}
	PUSH {R0-R3}
	
		LDR		R0, =SIM_SCGC6
		LDR		R1, =SIM_SCGC6_PIT_MASK
		LDR		R2,[R0,#0]
		ORRS	R2,R2,R1 ; ONLY PIT BIT SET
		STR		R2,[R0,#0]
		
		LDR		R0,=PIT_BASE
		LDR		R1,=PIT_MCR_EN_FRZ
		STR   	R1,[R0,#PIT_MCR_OFFSET]
		
		LDR		R0,=PIT_CH0_BASE
		LDR		R1,=PIT_LDVAL_10ms
		STR   	R1,[R0,#PIT_LDVAL_OFFSET]
		
		LDR		R0,=PIT_CH0_BASE
		MOVS	R1,#PIT_TCTRL_CH_IE
		STR		R1,[R0,#PIT_TCTRL_OFFSET]
		
		;unmask PIT interrupts
		LDR		R0,=NVIC_ISER
		LDR		R1,=PIT_IRQ_MASK
		STR		R1,[R0,#0]
		
		;set PIT interrupt priority
		LDR		R0,=PIT_IPR
		LDR		R1,=(NVIC_IPR_PIT_MASK)
		LDR		R2,=(PIT_IRQ_PRI << PIT_PRI_POS)
		LDR		R3,[R0,#0]
		BICS	R3,R3,R1
		ORRS	R3,R3,R2
		STR		R1,[R0,#0]
		POP		{R0-R3}
		BX		LR
		ENDP

;=============================================
PIT_ISR	    PROC {R1-R14}
			PUSH{R0-R2}
			
		LDR		R0,=RunStopWatch
		LDRB	R0,[R0,#0]
		CMP		R0,#0
		BEQ		INTRP_CLR
		
		LDR		R1,=Count
		LDR		R2,[R1,#0]
		ADDS	R2,R2,#1
		STR		R2,[R1,#0]

INTRP_CLR
		LDR		R0,=PIT_CH0_BASE
		LDR		R1,=PIT_TFLG_TIF_MASK
		STR		R1,[R0,#PIT_TFLG_OFFSET]
		
		POP		{R0-R2}
		BX		LR	
		ENDP
;=============================================
PutChar		PROC {R1-R14}
			PUSH	{R1,R2,R4}
			LDR		R1,=UART0_BASE
			MOVS	R2,#UART0_S1_TDRE_MASK
PollTx		LDRB	R4,[R1,#UART0_S1_OFFSET]
			ANDS	R4,R4,R2
			BEQ		PollTx
			STRB    R0,[R1,#UART0_D_OFFSET]
			POP	{R1,R2,R4}
			BX LR
			ENDP
;============================================
PutNumHex	PROC {R0-R14}
			PUSH {R0-R6,LR}
;MASK OFF FOUR BITS AT A TIME AND THEN OUTPUT THE BYTES ONE BY ONE
			MOVS	R4,R0
			MOVS	R3,R0
			MOVS	R5,#28
			LDR		R6,=0XF0000000
LOOP_Hex
			CMP		R6,#0
			BEQ		END_LOOP
			ANDS	R4,R4,R6
			LSRS	R4,R4,R5
			CMP		R4,#9
			BGT		HEX_LET
			ADDS	R4,R4,#'0'
			B		END_CONV

HEX_LET		
			ADDS	R4,R4,#55
END_CONV
			MOVS	R0,R4
			BL 		PutChar
			MOVS	R4,R3
			SUBS	R5,R5,#4
			LSRS	R6,R6,#4
			
			B		LOOP_Hex
END_LOOP
			POP 	{R0-R6,PC}
			BX		LR
			ENDP

;=============================================
InitQueue	PROC {R1-R14}
			PUSH {R1-R2}
			STR		R0,[R1,#IN_PTR]
			STR		R0,[R1,#OUT_PTR]
			STR		R0,[R1,#BUF_STRT]
			ADDS	R0,R0,R2
			STR		R0,[R1,#BUF_PAST]
			STRB	R2,[R1,#BUF_SIZE]
			MOVS	R0,#0
			STRB	R0,[R1,#NUM_ENQD]
			POP	{R1-R2}
			BX 		LR
			ENDP
				
;=============================================;
Dequeue 	PROC {R1-R14}
			PUSH {R1-R5}
	;Input R1 = address of the queue record structure
	;Output R0: character dequeued
	;APSR C flag: success (0) or failure(i)
	;Modify R0;APSR
		LDR		R6,[R1,#BUF_STRT]
		LDRB	R2,[R1,#NUM_ENQD]
		LDR	 	R3,[R1,#OUT_PTR]
		LDR		R5,[R1,#BUF_PAST]
;If queue is not empty ;NumberEnqueued>0
		CMP		R2,#0
		BEQ		End_Dequeue
;Get item at OutPointer
;Load r0 from OutPointeru
			LDRB 	R0,[R3,#0]
;Decrement NumberEnqueued
		SUBS	R2,R2,#1
		STRB	R2,[R1,#NUM_ENQD]
;Increment OutPointer
		ADDS	R3,R3,#1
		STR 	R3,[R1,#OUT_PTR]
;If OutPointer Points outside queue buffer ;OutPointer >= BufferPast 
		CMP		R3,R5
		BNE		Step
		STR		R6,[R1,#OUT_PTR]
Step
;Reflect Success in APSR 
		MRS		R4,APSR
		MOVS	R5,#2_11011111
		LSLS	R5,R5,#24
		ANDS	R4,R4,R5
		MSR		APSR,R4
		B		SKIPD
End_Dequeue
		MRS		R4,APSR
		MOVS	R5,#0X20
		LSLS	R5,R5,#24
		ORRS	R4,R4,R5
		MSR		APSR,R4
		B		SKIPD
SKIPD
		POP{R1-R5}
		BX LR
		ENDP
		
;=============================================
Enqueue	PROC {R1-R14}
	PUSH {R0-R6}
;Save on stack any register used other than APSR
;(The inpointer and number enqueued will change)
;(In pointer increases by one byte everytime a character is put in)
;If Queue is not full (number enqueued) {NumberEnqueued < BufferSize)
			;NumberEnqueued
			LDR		R5,[R1,#BUF_PAST]
			LDR		R4,[R1,#IN_PTR]
			LDR		R6,[R1,#BUF_STRT]
			LDRB	R2,[R1,#NUM_ENQD]
			LDRB	R3,[R1,#BUF_SIZE]
			;BufferSize
			CMP		R2,R3
			BGE		End_Enqueue
;Put new element at memory location pointed by InPointer
;Store R0 at InPointer
			STRB	R0,[R4,#0]
;Increment NumberEnqueued
			ADDS	R2,R2,#1
			STRB	R2,[R1,#NUM_ENQD]
;Increment InPointer past queue item
			ADDS	R4,R4,#1
			STR		R4,[R1,#IN_PTR]
;If InPointer points outside queue buffer, ;InPointer>=BufferPast ;LOAD BUFFERPAST AND CHECK
			
			CMP		R4,R5
			BLO		Step0
			STR		R6,[R1,#IN_PTR]
Step0
			MRS		R4,APSR
			MOVS	R5,#2_11011111
			LSLS	R5,R5,#24
			ANDS	R4,R4,R5
			MSR		APSR,R4
			B		End_ENQ
End_Enqueue
			MRS		R4,APSR
			MOVS	R5,#0X20
			LSLS	R5,R5,#24
			ORRS	R4,R4,R5
			MSR		APSR,R4
			
			
End_ENQ			
		POP {R0-R6}
		BX 	LR
		ENDP
			

;=============================================
DIVU  		PROC {R2-R14} 		;Register 2- Register 14 are unchanged
			PUSH {R2,R3}		;Register 2 and 3 values are stored in a stack
			MOVS 	R2,#0		;SettinG R2 to 0
			CMP 	R0,#0		;If the divisor is not 0, then the code can reach the while loop,or else, the code sets the C bit and exits
			BNE 	WHILE		;Go to while only if R0 is not 0
			MRS 	R2,APSR		;APSR stored into R2 for setting C bit
			MOVS	R3,#0X20	;Mask bit string created from R3 
			LSLS	R3,R3,#24	;The C bit (29th bit) is set to 1
			ORRS	R2,R2,R3	;ORRED with R2 so that C bit can be set
			MSR		APSR,R2		;R2 moved back to flags
			B		ENDPROG		;End to subroutine
			
			
WHILE		
			CMP	 R1,R0 			;Compared R1 and R0
			BLO  End_WHILE		;If R1 is smaller then while loop is ended
			SUBS R1,R1,R0 		;R0 is subtracted off of R1 
			ADDS R2,R2,#1		;R2 contains the quotient and is incremented everytime R0 is subtracted
			B WHILE				;Branch back to while
End_WHILE
			MOVS R0,R2 			;Quotient moved to R2
			ADDS R2,R2,#0		;C flag is set to 0 with this statement 
ENDPROG
			POP {R2,R3}			;Register Values R2 and R3 stored back to thier original spots from the stack
			BX 	LR				;The result stored in the LR register with R1 and R0 also changed
				ENDP
;=============================================;
PutStringSB PROC	{R1-R14}
			PUSH	{R1,R2,R3,LR}
			MOVS	R2,R0
			MOVS	R3,#0
WHILE_PS
			CMP		R3,R1
			BGE		END_WHILEPS
			LDRB	R0,[R2,R3]
			CMP		R0,#0
			BEQ		END_WHILEPS
			BL		PutChar
			ADDS	R3,R3,#1
			B		WHILE_PS
END_WHILEPS
			POP		{R1,R2,R3,PC}
			ENDP
;=============================================;

Init_UART0_Polling 	PROC	{R1-R14}
			PUSH	{R1,R2,R3}
			LDR 	R0,=SIM_SOPT2
			LDR		R1,=SIM_SOPT2_UART0SRC_MASK
			LDR		R2,[R0,#0]
			BICS	R2,R2,R1
			LDR		R1,=SIM_SOPT2_UART0SRC_MCGFLLCLK
			ORRS	R2,R2,R1
			STR		R2,[R0,#0]
			
			LDR		R0,=SIM_SCGC4
			LDR		R1,=SIM_SCGC4_UART0_MASK
			LDR		R2,[R0,#0]
			ORRS	R2,R2,R1
			STR		R2,[R0,#0]
			
			LDR		R0,=SIM_SOPT5
			LDR		R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
			LDR		R2,[R0,#0]
			
			LDR		R0,=SIM_SCGC5
			LDR		R1,=SIM_SCGC5_PORTB_MASK
			LDR		R2,[R0,#0]
			ORRS	R2,R2,R1
			STR		R2,[R0,#0]
			
			LDR		R0,=PORTB_PCR2
			LDR		R1,=PORT_PCR_SET_PTB2_UART0_RX
			STR		R1,[R0,#0]
			LDR		R0,=PORTB_PCR1
			LDR		R1,=PORT_PCR_SET_PTB1_UART0_TX
			STR		R1,[R0,#0]	
			;;;;;;;;;;;;;;;;;;;;;;;;;
			;REGISTER INITIALISATION;
			;;;;;;;;;;;;;;;;;;;;;;;;;
			LDR		R0,=UART0_BASE
			MOVS	R1,#UART0_C2_T_R
			LDRB	R2,[R0,#UART0_C2_OFFSET]
			BICS	R2,R2,R1
			STRB	R2,[R0,#UART0_C2_OFFSET]
			MOVS 	R1,#UART0_BDH_9600
			STRB	R1,[R0,#UART0_BDH_OFFSET]
			MOVS	R1,#UART0_BDL_9600
			STRB	R1,[R0,#UART0_BDL_OFFSET]
			
			;SET UART CHARACTER FORMAT FOR SERIAL BIT STREAM
			MOVS	R1,#UART0_C1_8N1
			STRB	R1,[R0,#UART0_C1_OFFSET]
			MOVS	R1,#UART0_C3_NO_TXINV
			STRB	R1,[R0,#UART0_C3_OFFSET]
			MOVS	R1,#UART0_C4_NO_MATCH_OSR_16
			STRB	R1,[R0,#UART0_C4_OFFSET]
			MOVS	R1,#UART0_C5_NO_DMA_SSR_SYNC
			STRB	R1,[R0,#UART0_C5_OFFSET]
			MOVS	R1,#UART0_S1_CLEAR_FLAGS
			STRB	R1,[R0,#UART0_S1_OFFSET]
			MOVS	R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
			STRB	R1,[R0,#UART0_S2_OFFSET]
			
			;ENABLE UART0
			MOVS	R1,#UART0_C2_T_R
			STRB	R1,[R0,#UART0_C2_OFFSET]
			POP 	{R1,R2,R3}
			BX LR
			ENDP	
;===================================================			

;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
            IMPORT  __initial_sp
            IMPORT  Dummy_Handler
            IMPORT  HardFault_Handler
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;01:reset vector
            DCD    Dummy_Handler      ;02:NMI
            DCD    HardFault_Handler  ;03:hard fault
            DCD    Dummy_Handler      ;04:(reserved)
            DCD    Dummy_Handler      ;05:(reserved)
            DCD    Dummy_Handler      ;06:(reserved)
            DCD    Dummy_Handler      ;07:(reserved)
            DCD    Dummy_Handler      ;08:(reserved)
            DCD    Dummy_Handler      ;09:(reserved)
            DCD    Dummy_Handler      ;10:(reserved)
            DCD    Dummy_Handler      ;11:SVCall (supervisor call)
            DCD    Dummy_Handler      ;12:(reserved)
            DCD    Dummy_Handler      ;13:(reserved)
            DCD    Dummy_Handler      ;14:PendSV (PendableSrvReq)
                                      ;   pendable request 
                                      ;   for system service)
            DCD    Dummy_Handler      ;15:SysTick (system tick timer)
            DCD    Dummy_Handler      ;16:DMA channel 0 transfer 
                                      ;   complete/error
            DCD    Dummy_Handler      ;17:DMA channel 1 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;18:DMA channel 2 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;19:DMA channel 3 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;20:(reserved)
            DCD    Dummy_Handler      ;21:FTFA command complete/
                                      ;   read collision
            DCD    Dummy_Handler      ;22:low-voltage detect;
                                      ;   low-voltage warning
            DCD    Dummy_Handler      ;23:low leakage wakeup
            DCD    Dummy_Handler      ;24:I2C0
            DCD    Dummy_Handler      ;25:(reserved)
            DCD    Dummy_Handler      ;26:SPI0
            DCD    Dummy_Handler      ;27:(reserved)
            DCD    UART0_ISR     ;28:UART0 (status; error);***************;
            DCD    Dummy_Handler      ;29:(reserved)
            DCD    Dummy_Handler      ;30:(reserved)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:(reserved)
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    PIT_ISR      ;38:PIT ;**********;
            DCD    Dummy_Handler      ;39:(reserved)
            DCD    Dummy_Handler      ;40:(reserved)
            DCD    Dummy_Handler      ;41:DAC0
            DCD    Dummy_Handler      ;42:TSI0
            DCD    Dummy_Handler      ;43:MCG
            DCD    Dummy_Handler      ;44:LPTMR0
            DCD    Dummy_Handler      ;45:(reserved)
            DCD    Dummy_Handler      ;46:PORTA
            DCD    Dummy_Handler      ;47:PORTB
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
Prompt		DCB		"Press key for stopwatch command (C,D,H,P,T):",NULL
Prompt_past

Help		DCB		"C(lear),D(isplay),H(elp),P(ause),T(ime)",NULL
Help_past

time_factor	DCB		" x 0.01s"
time_factor_past
		
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
Count			SPACE	4
RunStopWatch	SPACE 	1
;QBuffer		SPACE	Q_BUF_SZ	;Queue contents
			ALIGN
;QRecord		SPACE	Q_REC_SZ	;Queue managment record
;>>>>>   end variables here <<<<<
            ALIGN
            END
