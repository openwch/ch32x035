;
INCLUDE				PIOC_INC.ASM
;
;
					ORG   0X0000
					DW    0X0000
					JMP   MCU_START
					DW    0X0FFF
;

REC_FLAG			EQU   SFR_DATA_REG0  ;receive flag  bit[0]@IO select. bit[1]@receive data judge.
DELAY_CNT			EQU   SFR_DATA_REG1  ;delay count
BIT_NUM				EQU   SFR_DATA_REG2  ;bit nums
VAR          		EQU   SFR_DATA_REG3  ;byte nums	
DATA_BUF			EQU	  SFR_DATA_REG4  ;data buff
DATA				EQU   SFR_DATA_REG11 ;receive data


CLK_10:				NOP
					NOP
					NOP
					NOP
					NOP
					NOP
					RET				
;DELAY 50US
NEC_DELAY_50US:		MOVL  50
					JMP   DELAY_US 					
;DELAY 10US
NEC_DELAY_10US:		MOVL  10
					JMP   DELAY_US 					
;DELAY US
DELAY_US:			CALL  CLK_10   ;10 CLOCK @48M
					NOP
					CALL  CLK_10   ;10 CLOCK @48M
					NOP
					CALL  CLK_10   ;10 CLOCK @48M
					NOP
					CALL  CLK_10   ;10 CLOCK @48M
					NOP
					ADDL  0XFF
					NOP
					JNZ   DELAY_US
					RET
;					
;
NEC_RECEIVE:        WAITB WB_DATA_MW_SR_1
					MOV   SFR_CTRL_WR,A				
					BTSC  REC_FLAG,0				;SEL REC_PIN
					JMP   PIOLOT_CODE_1
					JMP   PIOLOT_CODE				
;return repreat code
RETURN_REPEAT:		MOVL  0X40						;REPEAT CODE
RETURN:				MOVA  SFR_CTRL_RD		        ;RETURN ERR DELAYCNT RESULT 
					BS    SFR_SYS_CFG,SB_INT_REQ	;REQUEST INTERRUPT
					BTSC  REC_FLAG,0
					JMP   PIOLOT_CODE_1
					NOP
; start code. PC18 or PC7
PIOLOT_CODE:		BS	  SFR_PORT_IO,SB_PORT_OUT0	;IO_0 HIGH
					BC    SFR_PORT_DIR,SB_PORT_DIR0	;IO_0 INPUT
					WAITB WB_PORT_XOR0_0			;WAIT IDLE IF HIGH
					CLR   DELAY_CNT
					MOVL  0X04
					MOVA  VAR
					MOVL  0X08
					MOVA  BIT_NUM
					MOVIP DATA
					WAITB WB_PORT_XOR0_1			;WAIT FALL
PIOL_LOW_KEEP:		CALL  NEC_DELAY_50US
					INC   DELAY_CNT
					BTSC  SFR_PORT_IO,SB_PORT_XOR0
					JMP   PIOL_LOW_KEEP
					MOV   DELAY_CNT,A
					CMPL  174
					JNC   RETURN					;174(8.7mS)-A >=0
					CMPL  188						
					JC    RETURN					;188(9.4mS)-A < 0
					CLR   DELAY_CNT
					WAITB WB_PORT_XOR0_0			
PIOL_HIGH_KEEP:		CALL  NEC_DELAY_50US
					INC   DELAY_CNT
					BTSS  SFR_PORT_IO,SB_PORT_XOR0
					JMP   PIOL_HIGH_KEEP	
					MOV   DELAY_CNT,A
					CMPL  38
					JNC   RETURN					;38(1.9mS)-A >=0
					CMPL  52
					JNC   RETURN_REPEAT				;52(2.6mS)-A >=0			
					CMPL  82
					JNC   RETURN					;84(4.1mS)-A >=0
					CMPL  96
					JC	  RETURN					;96(4.8mS)-A < 0
				    JMP   DATA_CODE				
; data code 	
DATA_CODE:          WAITB WB_PORT_XOR0_1	
					CLR   DELAY_CNT
					BC    REC_FLAG,1 				
					WAITB WB_PORT_XOR0_0			;WAIT RISE
DATA_HIGH_KEEP:		CALL  NEC_DELAY_10US		
					INC   DELAY_CNT
					BTSS  SFR_PORT_IO,SB_PORT_XOR0
					JMP   DATA_HIGH_KEEP	
					MOV   DELAY_CNT,A
					CMPL  30
					JNC   RETURN					;30(300uS)-A >=0
					CMPL  80
					JNC   REC_0						;80(800uS)-A >= 0  receive data 0. bit<0> 0--data 0  1--data 1			
					CMPL  130		  
					JNC   RETURN					;130(1300uS)-A >=0
					CMPL  190
					JC    RETURN					;190(1900uS)-A < 0
					BS    REC_FLAG,1				;receive data 1.
REC_0:				RCL   DATA_BUF
					BTSC  REC_FLAG,1
					INC   DATA_BUF 
					DEC   BIT_NUM
					BTSS  SFR_STATUS_REG,SB_FLAG_Z
					JMP   DATA_CODE
					MOVL  0X08
					MOVA  BIT_NUM
					MOV   DATA_BUF,A
					CLR   DATA_BUF
					MOVA  SFR_INDIR_PORT
					DEC   SFR_INDIR_ADDR
					DEC   VAR
					BTSS  SFR_STATUS_REG,SB_FLAG_Z
					JMP   DATA_CODE
					MOVL  0X80						;Received a frame of data
					JMP   RETURN
					NOP										
; start code. PC19
PIOLOT_CODE_1:		BC	  SFR_PORT_IO,SB_PORT_OUT1	;IO_1 LOW
					BC    SFR_PORT_DIR,SB_PORT_DIR1	;IO_1 INPUT
					WAITB WB_PORT_XOR1_1			;WAIT IDLE IF HIGH
					CLR   DELAY_CNT
					MOVL  0X04
					MOVA  VAR
					MOVL  0X08
					MOVA  BIT_NUM
					MOVIP DATA
					BS	  SFR_PORT_IO,SB_PORT_OUT1	;IO_1 HIGH
					WAITB WB_PORT_XOR1_1			;WAIT FALL
PIOL_LOW_KEEP_1:	CALL  NEC_DELAY_50US
					INC   DELAY_CNT
					BTSC  SFR_PORT_IO,SB_PORT_XOR1
					JMP   PIOL_LOW_KEEP_1
					MOV   DELAY_CNT,A
					CMPL  174						;174(8.7mS)-A >=0
					JNC   RETURN
					CMPL  188						;188(9.4mS)-A < 0
					JC    RETURN
					CLR   DELAY_CNT
					BC	  SFR_PORT_IO,SB_PORT_OUT1	;IO_1 LOW
					WAITB WB_PORT_XOR1_1			
PIOL_HIGH_KEEP_1:   CALL  NEC_DELAY_50US
					INC   DELAY_CNT
					BTSC  SFR_PORT_IO,SB_PORT_XOR1
					JMP   PIOL_HIGH_KEEP_1	
					MOV   DELAY_CNT,A
					CMPL  38
					JNC   RETURN					;38(1.9mS)-A >=0
					CMPL  52
					JNC   RETURN_REPEAT				;52(2.6mS)-A >=0			
					CMPL  82
					JNC   RETURN					;84(4.1mS)-A >=0
					CMPL  96
					JC	  RETURN					;96(4.8mS)-A < 0
				    JMP   DATA_CODE_1				
; data code 	
DATA_CODE_1:        BS	  SFR_PORT_IO,SB_PORT_OUT1	;IO_1 HIGH
					WAITB WB_PORT_XOR1_1	
					CLR   DELAY_CNT
					BC    REC_FLAG,1 
					BC	  SFR_PORT_IO,SB_PORT_OUT1	;IO_1 LOW
					WAITB WB_PORT_XOR1_1			;WAIT RISE
DATA_HIGH_KEEP_1:	CALL  NEC_DELAY_10US		
					INC   DELAY_CNT
					BTSC  SFR_PORT_IO,SB_PORT_XOR1
					JMP   DATA_HIGH_KEEP_1	
					MOV   DELAY_CNT,A
					CMPL  30
					JNC   RETURN					;30(300uS)-A >=0
					CMPL  80
					JNC   REC_0_1					;80(800uS)-A >= 0  receive data 0. bit<0> 0--data 0  1--data 1			
					CMPL  130		  
					JNC   RETURN					;130(1300uS)-A >=0
					CMPL  190
					JC    RETURN					;190(1900uS)-A < 0
					BS    REC_FLAG,1				;receive data 1.
REC_0_1:			RCL   DATA_BUF
					BTSC  REC_FLAG,1
					INC   DATA_BUF 
					DEC   BIT_NUM
					BTSS  SFR_STATUS_REG,SB_FLAG_Z
					JMP   DATA_CODE_1
					MOVL  0X08
					MOVA  BIT_NUM
					MOV   DATA_BUF,A
					CLR   DATA_BUF
					MOVA  SFR_INDIR_PORT
					DEC   SFR_INDIR_ADDR
					DEC   VAR
					BTSS  SFR_STATUS_REG,SB_FLAG_Z
					JMP   DATA_CODE_1
					MOVL  0X80						;Received a frame of data
					JMP   RETURN
					NOP	
;									
;					
MCU_START:			NOP
					NOP
					JMP   NEC_RECEIVE
					JMP   MCU_START
;
END
;

