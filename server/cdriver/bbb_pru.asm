#define TICK_US 40
	.origin 0
	.entrypoint start
start:
	; Set up IEP (counter).
	; Set increments.
	ldi r0, 0x0111
	sbco r0, c26, 0x0, 4
	; Clear overflow.
	ldi r0, 0x1
	sbco r0, c26, 0x4, 4
	; Enable cmp0 and reset on overflow.
	ldi r0, 0x3
	sbco r0, c26, 0x40, 4
	; Clear hit status.
	ldi r0, 0xff
	sbco r0, c26, 0x44, 4
	; Set cmp0 value.
	mov r0, 200
	sbco r0, c26, 0x48, 4

	; Set up intc.
	; Enable interrupts.
	ldi r0, 0x1
	sbco r0, c0, 0x10, 4
	; Enable IEP counter interrupt.
	ldi r0, 0x7
	sbco r0, c0, 0x28, 4
	; Clear the interrupt.
	sbco r0, c0, 0x24, 4
	; Enable host output for channel 0.
	ldi r0, 0x0
	sbco r0, c0, 0x34, 4
	; Map IEP interrupt to channel 0.
	ldi r1, 0x404
	sbco r0, c0, r1, 4
	; Map channels.
	ldi r0, 0x00
	ldi r1, 0x800
	sbco r0, c0, r1, 1

	; Setup done, simple test: blink an output.
	mov r0, 0
	mov r1, 1
	mov r2, 7
	mov r5, TICK_US - 3

	.macro wait_for_tick
	wbs r31, 30		; wait
	sbco r2, c26, 0x44, 4	; clear timer flag
	sbco r0, c0, 0x24, 4	; clear interrupt flag
	.endm

mainloop:
	lbbo r3, c24, 0, 8	; load current settings
	; r3.w0 = neg_base
	; r3.w1 = pos_base
	; r4.b0 = current_sample
	; r4.b1 = current_fragment
	; r4.b2 = next_fragment
	; r4.b3 = state

	; output neg_base
	mov r30.w0, r3.w0
	; if state < 2: continue
	qbeq mainloop, r4.b3, 2
	; if state == 4: state = 1; continue
	qbeq skip1, r4.b3, 4
	sbco r1, c24, 8, 1
	qba mainloop
skip1:
	; wait for enough time to allow next tick.
	wait_for_tick
	sub r5, r5, 1
	qbne mainloop, r5, 0
	mov r5, TICK_US - 3

	; data is at buffer[fragment][sample][which] with sample array 256 elements, which array 2 elements and 2 bytes per element.
	; So that's buffer_start + fragment * 256 * 2 * 2 + sample * 2 * 2 + which * 2; I want both which values.
	lsl r6, r4.b1, 12
	lsl r7, r4.b0, 4
	add r6, r6, r7
	add r6, r6, 8	; buffer start.
	lbco r7, c24, r6, 4

	; do step
	mov r30.w0, r7.w0
	wait_for_tick
	mov r30.w0, r3.w0
	wait_for_tick
	mov r30.w0, r7.w1
	wait_for_tick
	mov r30.w0, r3.w0

	; next sample
	add r4.b0, r4.b0, 1
	qbne skip2, r4.b0, 0
	; next fragment
	add r4.b1, r4.b1, 1
	and r4.b1, r4.b1, 0x7
	; underrun
	qbne skip2, r4.b1, r4.b2
	mov r4.b3, 1
skip2:
	sbco r4, c24, 0, 4
	; if state == 2: state = 0
	qbne mainloop, r4.b3, 4
	sbco r0, c24, 8, 1
	; continue
	qba mainloop
