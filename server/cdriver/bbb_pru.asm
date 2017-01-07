; bbb-pru.asm - programmable realtime unit firmware for Franklin on BeagleBone.
; Copyright 2014-2016 Michigan Technological University
; Copyright 2016 Bas Wijnen <wijnen@debian.org>
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

#include "pru.asm"

#define TICK_US 40

	counter_set_increments 1, 1
	counter_set_cmp 0, 200	; one interrupt per microsecond
	counter_set_cmp0_top
	counter_enable

	; Setup done.
	mov r0, 0
	mov r1, 1
	mov r2, 7
	mov r5, TICK_US - 5

	.macro wait_for_tick
wait_loop:
	counter_get_hit
	qbbc wait_loop, TMP, 0
	counter_clear_hit 0
	.endm

mainloop:
	lbco r3, CONST_OWN_DATA, 0, 8	; load current settings

	; r3.w0 = base
	; r3.w2 = dirs
	; r4.b0 = current_sample
	; r4.b1 = current_fragment
	; r4.b2 = next_fragment
	; r4.b3 = state

	; output base
	wait_for_tick
	mov r30.w0, r3.w0
	; if 2 > state: continue
	qbgt mainloop, r4.b3, 2
	; if state == 4: state = 1; continue
	qbne skip1, r4.b3, 4
	sbco r1.b0, CONST_OWN_DATA, 7, 1
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
	lbco r7, CONST_OWN_DATA, r6, 4
	xor r3.w2, r3.w2, r3.w0	; Apply base to dirs
	xor r7.w0, r7.w0, r3.w0	; Apply base to neg
	xor r7.w2, r7.w2, r3.w2	; Apply base+dirs to pos

	; do step
	wait_for_tick
	mov r30.w0, r7.w0
	wait_for_tick
	mov r30.w0, r3.w0
	wait_for_tick
	mov r30.w0, r3.w2
	wait_for_tick
	mov r30.w0, r7.w2
	wait_for_tick
	mov r30.w0, r3.w2
	; r3.w0 is sent in the next loop iteration.

	; next sample
	add r4.w0, r4.w0, 1
	qbne skip2, r4.b0, 0
	; next fragment
	and r4.b1, r4.b1, 0x7
	; underrun
	qbne skip2, r4.b1, r4.b2
	mov r4.b3, 1
skip2:
	sbco r4, CONST_OWN_DATA, 4, 4
	; if state == 2: state = 0
	qbne mainloop, r4.b3, 2
	sbco r0.b0, CONST_OWN_DATA, 7, 1
	; continue
	qba mainloop
