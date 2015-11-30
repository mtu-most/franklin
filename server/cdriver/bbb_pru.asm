; bbb-pru.asm - programmable realtime unit firmware for Franklin on BeagleBone.
; Copyright 2014 Michigan Technological University
; Author: Bas Wijnen <wijnen@debian.org>
;
; This program is free software: you can redistribute it and/or modify
; it under the terms of the GNU Affero General Public License as
; published by the Free Software Foundation, either version 3 of the
; License, or (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU Affero General Public License for more details.
;
; You should have received a copy of the GNU Affero General Public License
; along with this program.  If not, see <http://www.gnu.org/licenses/>.

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

	; Setup done.
	mov r0, 0
	mov r1, 1
	mov r2, 7
	mov r5, TICK_US - 5

	.macro wait_for_tick
	wbs r31, 30		; wait
	sbco r2, c26, 0x44, 4	; clear timer flag
	sbco r0, c0, 0x24, 4	; clear interrupt flag
	.endm

mainloop:
	lbco r3, c24, 0, 8	; load current settings

	; r3.w0 = base
	; r3.w1 = dirs
	; r4.b0 = current_sample
	; r4.b1 = current_fragment
	; r4.b2 = next_fragment
	; r4.b3 = state

	; output base
	wait_for_tick
	mov r30.w0, r3.w0
	; if state < 2: continue
	qbge mainloop, r4.b3, 2
	; if state == 4: state = 1; continue
	qbne skip1, r4.b3, 4
	sbco r1.b0, c24, 7, 1
	qba mainloop
skip1:

	sub r5, r5, 1
	; wait for enough time to allow next tick.
	wait_for_tick
	qbne mainloop, r5, 0
	mov r5, TICK_US - 5

	; data is at buffer[fragment][sample][which] with sample array 256 elements, which array 2 elements and 2 bytes per element.
	; So that's buffer_start + fragment * 256 * 2 * 2 + sample * 2 * 2 + which * 2; I want both which values.
	lsl r6, r4.b1, 10
	lsl r7, r4.b0, 2
	add r6, r6, r7
	add r6, r6, 8	; buffer start.
	lbco r7, c24, r6, 4
	xor r3.w1, r3.w1, r3.w0	; Apply base to dirs
	xor r7.w0, r7.w0, r3.w0	; Apply base to neg
	xor r7.w1, r7.w1, r3.w1	; Apply base+dirs to pos

	; do step
	wait_for_tick
	mov r30.w0, r7.w0
	wait_for_tick
	mov r30.w0, r3.w0
	wait_for_tick
	mov r30.w0, r3.w1
	wait_for_tick
	mov r30.w0, r7.w1
	wait_for_tick
	mov r30.w0, r3.w1
	; r3.w0 is sent in the next loop iteration.

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
	sbco r4, c24, 4, 4
	; if state == 2: state = 0
	qbne mainloop, r4.b3, 2
	sbco r0.b0, c24, 7, 1
	; continue
	qba mainloop
