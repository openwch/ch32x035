; include file for PIOC/eMCU, V1.0
; by W.ch @2022.08
; http://wch.cn  http://winchiphead.com
;

; define SFR register
SFR_INDIR_PORT      EQU   0x00
SFR_INDIR_PORT2     EQU   0x01
SFR_PRG_COUNT       EQU   0x02
SFR_STATUS_REG      EQU   0x03
SFR_INDIR_ADDR      EQU   0x04
SFR_TMR0_COUNT      EQU   0x05
SFR_TIMER_CTRL      EQU   0x06
SFR_TMR0_INIT       EQU   0x07
SFR_BIT_CYCLE       EQU   0x08
SFR_INDIR_ADDR2     EQU   0x09
SFR_PORT_DIR        EQU   0x0A
SFR_PORT_IO         EQU   0x0B
SFR_BIT_CONFIG      EQU   0x0C
SFR_SYS_CFG         EQU   0x1C
SFR_CTRL_RD         EQU   0x1D
SFR_CTRL_WR         EQU   0x1E
SFR_DATA_EXCH       EQU   0x1F
SFR_DATA_REG0       EQU   0x20
SFR_DATA_REG1       EQU   0x21
SFR_DATA_REG2       EQU   0x22
SFR_DATA_REG3       EQU   0x23
SFR_DATA_REG4       EQU   0x24
SFR_DATA_REG5       EQU   0x25
SFR_DATA_REG6       EQU   0x26
SFR_DATA_REG7       EQU   0x27
SFR_DATA_REG8       EQU   0x28
SFR_DATA_REG9       EQU   0x29
SFR_DATA_REG10      EQU   0x2A
SFR_DATA_REG11      EQU   0x2B
SFR_DATA_REG12      EQU   0x2C
SFR_DATA_REG13      EQU   0x2D
SFR_DATA_REG14      EQU   0x2E
SFR_DATA_REG15      EQU   0x2F
SFR_DATA_REG16      EQU   0x30
SFR_DATA_REG17      EQU   0x31
SFR_DATA_REG18      EQU   0x32
SFR_DATA_REG19      EQU   0x33
SFR_DATA_REG20      EQU   0x34
SFR_DATA_REG21      EQU   0x35
SFR_DATA_REG22      EQU   0x36
SFR_DATA_REG23      EQU   0x37
SFR_DATA_REG24      EQU   0x38
SFR_DATA_REG25      EQU   0x39
SFR_DATA_REG26      EQU   0x3A
SFR_DATA_REG27      EQU   0x3B
SFR_DATA_REG28      EQU   0x3C
SFR_DATA_REG29      EQU   0x3D
SFR_DATA_REG30      EQU   0x3E
SFR_DATA_REG31      EQU   0x3F

; define bit for SFR_STATUS_REG
SB_EN_TOUT_RST      EQU   5
SB_STACK_USED       EQU   4
SB_GP_BIT_Y         EQU   3
SB_FLAG_Z           EQU   2
SB_GP_BIT_X         EQU   1
SB_FLAG_C           EQU   0

; define bit for SFR_TIMER_CTRL
SB_EN_LEVEL1        EQU   7
SB_EN_LEVEL0        EQU   6
SB_TMR0_ENABLE      EQU   5
SB_TMR0_OUT_EN      EQU   4
SB_TMR0_MODE        EQU   3
SB_TMR0_FREQ2       EQU   2
SB_TMR0_FREQ1       EQU   1
SB_TMR0_FREQ0       EQU   0

; define bit for SFR_BIT_CYCLE
SB_BIT_TX_O0        EQU   7
SB_BIT_CYCLE_6      EQU   6
SB_BIT_CYCLE_5      EQU   5
SB_BIT_CYCLE_4      EQU   4
SB_BIT_CYCLE_3      EQU   3
SB_BIT_CYCLE_2      EQU   2
SB_BIT_CYCLE_1      EQU   1
SB_BIT_CYCLE_0      EQU   0

; define bit for SFR_PORT_DIR
SB_PORT_MOD3        EQU   7
SB_PORT_MOD2        EQU   6
SB_PORT_MOD1        EQU   5
SB_PORT_MOD0        EQU   4
SB_PORT_PU1         EQU   3
SB_PORT_PU0         EQU   2
SB_PORT_DIR1        EQU   1
SB_PORT_DIR0        EQU   0

; define bit for SFR_PORT_IO
SB_PORT_IN_XOR      EQU   7
SB_BIT_RX_I0        EQU   6
SB_PORT_IN1         EQU   5
SB_PORT_IN0         EQU   4
SB_PORT_XOR1        EQU   3
SB_PORT_XOR0        EQU   2
SB_PORT_OUT1        EQU   1
SB_PORT_OUT0        EQU   0

; define bit for SFR_BIT_CONFIG
SB_BIT_TX_EN        EQU   7
SB_BIT_CODE_MOD     EQU   6
SB_PORT_IN_EDGE     EQU   5
SB_BIT_CYC_TAIL     EQU   4
SB_BIT_CYC_CNT6     EQU   3
SB_BIT_CYC_CNT5     EQU   2
SB_BIT_CYC_CNT4     EQU   1
SB_BIT_CYC_CNT3     EQU   0

; define bit for SFR_SYS_CFG
SB_INT_REQ          EQU   7
SB_DATA_SW_MR       EQU   6
SB_DATA_MW_SR       EQU   5
SB_MST_CFG_B4       EQU   4
SB_MST_IO_EN1       EQU   3
SB_MST_IO_EN0       EQU   2
SB_MST_RESET        EQU   1
SB_MST_CLK_GATE     EQU   0

; define inform for BCTC instruction
BI_C_XOR_IN0        EQU   0

; define inform for BP1F/BP2F/BG1F/BG2F instruction
BIO_FLAG_C          EQU   0

; define inform for BCTC/BG1F/BG2F instruction
BI_BIT_RX_I0        EQU   1
BI_PORT_IN0         EQU   2
BI_PORT_IN1         EQU   3

; define inform for BP1F/BP2F instruction
BO_BIT_TX_O0        EQU   1
BO_PORT_OUT0        EQU   2
BO_PORT_OUT1        EQU   3

; define inform for WAITB instruction
WB_DATA_SW_MR_0     EQU   0
WB_BIT_CYC_TAIL_1   EQU   1
WB_PORT_I0_FALL     EQU   2
WB_PORT_I0_RISE     EQU   3
WB_DATA_MW_SR_1     EQU   4
WB_PORT_XOR1_1      EQU   5
WB_PORT_XOR0_0      EQU   6
WB_PORT_XOR0_1      EQU   7
