; This file contains some useful macros for setting up built in features of the PRU.

#define TMP r29
#define TMP2 r28	; Only used during init.

#define CONST_INTC		c0
#define CONST_PRU_CTRL		c4
#define CONST_UART0		c7
#define CONST_OWN_DATA		c24
#define CONST_OTHER_DATA	c25
#define CONST_IEP		c26
#define CONST_SHARED_DATA	c28

#define CTRL_CTBIR0	0x20

#ifndef NO_HEADER
	.origin 0
	.entrypoint start
start:
	mov TMP, 0
	mov TMP2, 0x22000
	sbbo TMP, TMP2, CTRL_CTBIR0, 4
	mov TMP2, 0x24000
	sbbo TMP, TMP2, CTRL_CTBIR0, 4
#endif

; INTC {{{
; Register definitions. {{{
#define INT_GER 0x10	; Global interrupt enable
#define INT_SISR 0x34	; System status indexed set
#define INT_SICR 0x24	; System status indexed clear
#define INT_EISR 0x28	; System enable indexed set
#define INT_EICR 0x2c	; System enable indexed set
#define INT_HIEISR 0x34	; Host enable indexed set
#define INT_HIEICR 0x38	; Host enable indexed clear
#define INT_SRSR0 0x200	; Current raw status of channels
#define INT_SRSR1 0x204	; Current raw status of channels
#define INT_SECR0 0x200	; Current status of channels
#define INT_SECR1 0x204	; Current status of channels
#define INT_ECR0 0x380	; Current status of channels
#define INT_ECR1 0x384	; Current status of channels
#define INT_HMR0 0x800	; Channel to host mapping
#define INT_HMR1 0x804	; Channel to host mapping
#define INT_HMR2 0x808	; Channel to host mapping
#define INT_HIER 0x1500	; 
; }}}
#define INT_EIP 7

#ifndef NO_INT_INIT ; {{{
	; Map channel X to host interrupt X for all channels.
	mov TMP2, INT_HMR0
	mov TMP, 0x3210
	sbco TMP, CONST_INTC, TMP2, 4
	add TMP2, TMP2, 4
	mov TMP, 0x7654
	sbco TMP, CONST_INTC, TMP2, 4
	add TMP2, TMP2, 4
	mov TMP, 0x0098
	sbco TMP, CONST_INTC, TMP2, 4
	; All sources are already mapped to channel 0, which is good.
	; Enable host interrupt 0, disable all others.
	mov TMP, 1
	mov TMP2, INT_HIER
	sbco TMP, CONST_INTC, TMP2, 4
	; Set global enable.
	mov TMP, 1
	sbco TMP, CONST_INTC, INT_GER, 1
	; Disable and clear all interrupts.
	mov TMP, 0xffffffff
	mov TMP2, INT_ECR0
	sbco TMP, CONST_INTC, TMP2, 4
	mov TMP2, INT_ECR1
	sbco TMP, CONST_INTC, TMP2, 4
	mov TMP2, INT_SECR0
	sbco TMP, CONST_INTC, TMP2, 4
	mov TMP2, INT_SECR1
	sbco TMP, CONST_INTC, TMP2, 4
#endif ; }}}

; Enable interrupt system
	.macro int_enable_channel	; {{{
	.mparam channel
	mov TMP, channel
	sbco TMP, CONST_INTC, INT_EISR, 4	; enable
	sbco TMP, CONST_INTC, INT_SICR, 4	; clear status
	.endm	; }}}
	.macro int_disable_channel	; {{{
	.mparam channel
	mov TMP, channel
	sbco TMP, CONST_INTC, INT_EICR, 4
	.endm	; }}}
	.macro int_clear_channel	; {{{
	.mparam channel
	mov TMP, (channel)
	sbco TMP, CONST_INTC, INT_SICR, 4
	.endm	; }}}
; }}}

; UART (TODO) {{{
; }}}

; IEP (timer) {{{
; Register definitions. {{{
#define IEP_GLOBAL_CFG		0x00
#define IEP_GLOBAL_STATUS	0x04
#define IEP_COMPEN		0x08
#define IEP_COUNT		0x0c
#define IEP_CMP_CFG		0x40
#define IEP_CMP_STATUS		0x44
#define IEP_CMP0		0x48
#define IEP_CMP1		0x4c
#define IEP_CMP2		0x50
#define IEP_CMP3		0x54
#define IEP_CMP4		0x58
#define IEP_CMP5		0x5c
#define IEP_CMP6		0x60
#define IEP_CMP7		0x64
; }}}

	.macro counter_enable	; {{{
	lbco TMP, CONST_IEP, IEP_GLOBAL_CFG, 1
	or TMP, TMP, 0x01
	sbco TMP, CONST_IEP, IEP_GLOBAL_CFG, 1
	.endm	; }}}
	.macro counter_disable	; {{{
	lbco TMP, CONST_IEP, IEP_GLOBAL_CFG, 1
	and TMP.b0, TMP.b0, 0xfe
	sbco TMP, CONST_IEP, IEP_GLOBAL_CFG, 1
	.endm	; }}}
	.macro counter_set_increments	; {{{
	.mparam normal, compensation
	lbco TMP, CONST_IEP, IEP_GLOBAL_CFG, 1
	and TMP, TMP, 0x01
	or TMP.b0, TMP.b0, (normal) << 4
	mov TMP.b1, (compensation)
	sbco TMP, CONST_IEP, IEP_GLOBAL_CFG, 2
	.endm	; }}}
	.macro counter_check_overflow	; {{{
	.mparam target=TMP
	lbco target, CONST_IEP, IEP_GLOBAL_STATUS, 1
	.endm	; }}}
	.macro counter_clear_overflow	; {{{
	mov TMP, 1
	sbco TMP, CONST_IEP, IEP_GLOBAL_STATUS, 1
	.endm	; }}}
	.macro counter_enable_compensation	; {{{
	.mparam times
	mov TMP, (times)
	sbco TMP, CONST_IEP, IEP_COMPEN, 3
	.endm	; }}}
	.macro counter_get	; {{{
	.mparam target=TMP
	lbco target, CONST_IEP, IEP_COUNT, 4
	.endm	; }}}
	.macro counter_set	; {{{
	.mparam value=0
	mov TMP, (value)
	sbco TMP, CONST_IEP, IEP_COUNT, 4
	.endm	; }}}
	.macro counter_enable_cmp	; {{{
	.mparam num
	; Note that this doesn't work for CMP7
	lbco TMP, CONST_IEP, IEP_CMP_CFG, 1
	or TMP.b0, TMP.b0, 1 << ((num) + 1)
	sbco TMP, CONST_IEP, IEP_CMP_CFG, 1
	.endm	; }}}
	.macro counter_disable_cmp	; {{{
	.mparam num
	; Note that this doesn't work for CMP7
	lbco TMP, CONST_IEP, IEP_CMP_CFG, 1
	and TMP.b0, TMP.b0, (1 << ((num) + 1)) ^ 0xff
	sbco TMP, CONST_IEP, IEP_CMP_CFG, 1
	.endm	; }}}
	.macro counter_set_cmp0_top	; {{{
	lbco TMP, CONST_IEP, IEP_CMP_CFG, 1
	or TMP, TMP, 0x03	; also enable cmp0
	sbco TMP, CONST_IEP, IEP_CMP_CFG, 1
	counter_set	; Set counter to 0, to be sure.
	.endm	; }}}
	.macro counter_unset_cmp0_top	; {{{
	lbco TMP, CONST_IEP, IEP_CMP_CFG, 1
	and TMP.b0, TMP.b0, 0xfe
	sbco TMP, CONST_IEP, IEP_CMP_CFG, 1
	.endm	; }}}
	.macro counter_get_hit	; {{{
	.mparam target=TMP
	lbco target, CONST_IEP, IEP_CMP_STATUS, 1
	.endm	; }}}
	.macro counter_clear_hit	; {{{
	.mparam num
	mov TMP, 1 << (num)
	sbco TMP, CONST_IEP, IEP_CMP_STATUS, 1
	lbco TMP, CONST_IEP, IEP_CMP_STATUS, 1	; Reading this seems to be required for resetting the interrupt.
	.endm	; }}}
	.macro counter_set_cmp	; {{{
	.mparam num, value
	mov TMP, (value)
	sbco TMP, CONST_IEP, IEP_CMP0 + (num) * 4, 4
	.endm	; }}}
; }}}

; vim: set foldmethod=marker :
