


.syntax unified
.thumb
.cpu cortex-m7
.fpu fpv5-sp-d16


/* =========================================================
 * ONLY PUBLIC API (VISIBLE FROM C)
 * ========================================================= */
.global BLDC_DriveCommutation
.global BLDC_RegenCommutation


/* =========================================================
 * TIM1 BASE
 * ========================================================= */
.equ TIM1_BASE, 0x40010000
.equ TIM1_CCER, 0x20
.equ TIM1_CCR1, 0x34
.equ TIM1_CCR2, 0x38
.equ TIM1_CCR3, 0x3C
.equ TIM1_ARR,  0x2C


/* =========================================================
 * TABLES
 * ========================================================= */
.section .rodata
.align 4

drive_table:
    .word case1, case2, case3, case4, case5, case6

regen_table:
    .word r1, r2, r3, r4, r5, r6


/* =========================================================
 * MACRO
 * ========================================================= */
.macro TIM1_LOAD
    ldr r0, =TIM1_BASE
.endm


/* =========================================================
 * PHASE A
 * ========================================================= */

.thumb_func
Phaze_A_ON:
    TIM1_LOAD
    ldr r1, [r0, #TIM1_CCER]
    orr r1, r1, #1
    bic r1, r1, #(1 << 1)
    str r1, [r0, #TIM1_CCER]
    bx lr

.thumb_func
Phaze_A_OFF:
    TIM1_LOAD
    ldr r1, [r0, #TIM1_CCER]
    bic r1, r1, #1
    orr r1, r1, #(1 << 1)
    str r1, [r0, #TIM1_CCER]
    bx lr

.thumb_func
Phaze_A_ZZ:
    TIM1_LOAD
    mov r1, #0
    str r1, [r0, #TIM1_CCER]
    bx lr

.thumb_func
Phaze_A_PWM:
    TIM1_LOAD
    ldr r1, [r0, #TIM1_CCER]
    orr r1, r1, #3
    str r1, [r0, #TIM1_CCER]
    bx lr

.thumb_func
Phaze_A_LOW:
    TIM1_LOAD
    mov r1, #0
    str r1, [r0, #TIM1_CCR1]
    bx lr

.thumb_func
Phaze_A_HIGH:
    TIM1_LOAD
    ldr r1, [r0, #TIM1_ARR]
    str r1, [r0, #TIM1_CCR1]
    bx lr


/* =========================================================
 * PHASE B
 * ========================================================= */

.thumb_func
Phaze_B_ON:
    TIM1_LOAD
    ldr r1, [r0, #TIM1_CCER]
    orr r1, r1, #(1 << 4)
    bic r1, r1, #(1 << 5)
    str r1, [r0, #TIM1_CCER]
    bx lr

.thumb_func
Phaze_B_OFF:
    TIM1_LOAD
    ldr r1, [r0, #TIM1_CCER]
    bic r1, r1, #(1 << 4)
    orr r1, r1, #(1 << 5)
    str r1, [r0, #TIM1_CCER]
    bx lr

.thumb_func
Phaze_B_ZZ:
    TIM1_LOAD
    mov r1, #0
    str r1, [r0, #TIM1_CCER]
    bx lr

.thumb_func
Phaze_B_PWM:
    TIM1_LOAD
    ldr r1, [r0, #TIM1_CCER]
    orr r1, r1, #(1 << 4)
    orr r1, r1, #(1 << 5)
    str r1, [r0, #TIM1_CCER]
    bx lr

.thumb_func
Phaze_B_LOW:
    TIM1_LOAD
    mov r1, #0
    str r1, [r0, #TIM1_CCR2]
    bx lr

.thumb_func
Phaze_B_HIGH:
    TIM1_LOAD
    ldr r1, [r0, #TIM1_ARR]
    str r1, [r0, #TIM1_CCR2]
    bx lr


/* =========================================================
 * PHASE C
 * ========================================================= */

.thumb_func
Phaze_C_ON:
    TIM1_LOAD
    ldr r1, [r0, #TIM1_CCER]
    orr r1, r1, #(1 << 8)
    bic r1, r1, #(1 << 9)
    str r1, [r0, #TIM1_CCER]
    bx lr

.thumb_func
Phaze_C_OFF:
    TIM1_LOAD
    ldr r1, [r0, #TIM1_CCER]
    bic r1, r1, #(1 << 8)
    orr r1, r1, #(1 << 9)
    str r1, [r0, #TIM1_CCER]
    bx lr

.thumb_func
Phaze_C_ZZ:
    TIM1_LOAD
    mov r1, #0
    str r1, [r0, #TIM1_CCER]
    bx lr

.thumb_func
Phaze_C_PWM:
    TIM1_LOAD
    ldr r1, [r0, #TIM1_CCER]
    orr r1, r1, #(1 << 8)
    orr r1, r1, #(1 << 9)
    str r1, [r0, #TIM1_CCER]
    bx lr

.thumb_func
Phaze_C_LOW:
    TIM1_LOAD
    mov r1, #0
    str r1, [r0, #TIM1_CCR3]
    bx lr

.thumb_func
Phaze_C_HIGH:
    TIM1_LOAD
    ldr r1, [r0, #TIM1_ARR]
    str r1, [r0, #TIM1_CCR3]
    bx lr


/* =========================================================
 * DRIVE TABLE FUNCTIONS
 * ========================================================= */

.thumb_func
case1:
    push {lr}
    bl Phaze_A_ON
    bl Phaze_B_ZZ
    bl Phaze_C_OFF
    pop {pc}

.thumb_func
case2:
    push {lr}
    bl Phaze_A_OFF
    bl Phaze_B_ON
    bl Phaze_C_ZZ
    pop {pc}

.thumb_func
case3:
    push {lr}
    bl Phaze_A_ZZ
    bl Phaze_B_ON
    bl Phaze_C_OFF
    pop {pc}

.thumb_func
case4:
    push {lr}
    bl Phaze_A_ZZ
    bl Phaze_B_OFF
    bl Phaze_C_ON
    pop {pc}

.thumb_func
case5:
    push {lr}
    bl Phaze_A_ON
    bl Phaze_B_OFF
    bl Phaze_C_ZZ
    pop {pc}

.thumb_func
case6:
    push {lr}
    bl Phaze_A_OFF
    bl Phaze_B_ZZ
    bl Phaze_C_ON
    pop {pc}


/* =========================================================
 * REGEN TABLE FUNCTIONS
 * ========================================================= */

.thumb_func
r1:
    push {lr}
    bl Phaze_A_PWM
    bl Phaze_B_ZZ
    bl Phaze_C_LOW
    pop {pc}

.thumb_func
r2:
    push {lr}
    bl Phaze_A_LOW
    bl Phaze_B_PWM
    bl Phaze_C_ZZ
    pop {pc}

.thumb_func
r3:
    push {lr}
    bl Phaze_A_ZZ
    bl Phaze_B_PWM
    bl Phaze_C_LOW
    pop {pc}

.thumb_func
r4:
    push {lr}
    bl Phaze_A_ZZ
    bl Phaze_B_LOW
    bl Phaze_C_PWM
    pop {pc}

.thumb_func
r5:
    push {lr}
    bl Phaze_A_PWM
    bl Phaze_B_LOW
    bl Phaze_C_ZZ
    pop {pc}

.thumb_func
r6:
    push {lr}
    bl Phaze_A_LOW
    bl Phaze_B_ZZ
    bl Phaze_C_PWM
    pop {pc}


/* =========================================================
 * MOTOR COMMUTATION (ONLY PUBLIC API #1)
 * ========================================================= */
.thumb_func
.global BLDC_DriveCommutation
.type BLDC_DriveCommutation, %function
BLDC_DriveCommutation:
    push {r4, lr}
    mov r4, r0

    cmp r4, #1
    blt zero_drive
    cmp r4, #6
    bgt zero_drive

    sub r4, r4, #1
    ldr r1, =drive_table
    ldr r2, [r1, r4, LSL #2]
    blx r2

    pop {r4, pc}

zero_drive:
    bl Phaze_A_ZZ
    bl Phaze_B_ZZ
    bl Phaze_C_ZZ
    pop {r4, pc}

/* =========================================================
 * REGEN COMMUTATION (ONLY PUBLIC API #2)
 * ========================================================= */
.thumb_func
.global BLDC_RegenCommutation
.type BLDC_RegenCommutation, %function
BLDC_RegenCommutation:
    push {r4, lr}
    mov r4, r0

    cmp r4, #1
    blt zero_regen
    cmp r4, #6
    bgt zero_regen

    sub r4, r4, #1
    ldr r1, =regen_table
    ldr r2, [r1, r4, LSL #2]
    blx r2

    pop {r4, pc}

zero_regen:
    bl Phaze_A_ZZ
    bl Phaze_B_ZZ
    bl Phaze_C_ZZ
    pop {r4, pc}

