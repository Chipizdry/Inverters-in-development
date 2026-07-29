
/*
    .syntax unified
    .thumb
    .text

    .global BLDC_MotorCommutation
    .thumb_func
    .type BLDC_MotorCommutation, %function

    // Объявляем внешние C-функции
    .extern Phaze_A_ON
    .extern Phaze_A_OFF
    .extern Phaze_A_ZZ
    .extern Phaze_B_ON
    .extern Phaze_B_OFF
    .extern Phaze_B_ZZ
    .extern Phaze_C_ON
    .extern Phaze_C_OFF
    .extern Phaze_C_ZZ

BLDC_MotorCommutation:
    push {r4-r7, lr}           // сохраняем регистры

    mov r4, r0                 // halls -> r4

    // Проверяем, если halls == 0 или 7 -> обнуление всех фаз
    cmp r4, #0
    beq _zero_phases
    cmp r4, #7
    beq _zero_phases

    // Основная логика: каждая комбинация вызывает функции HAL
    cmp r4, #1
    beq _case1
    cmp r4, #2
    beq _case2
    cmp r4, #3
    beq _case3
    cmp r4, #4
    beq _case4
    cmp r4, #5
    beq _case5
    cmp r4, #6
    beq _case6
    b _end

_case1:
    bl Phaze_A_ON
    bl Phaze_B_ZZ
    bl Phaze_C_OFF
    b _end

_case2:
    bl Phaze_A_OFF
    bl Phaze_B_ON
    bl Phaze_C_ZZ
    b _end

_case3:
    bl Phaze_A_ZZ
    bl Phaze_B_ON
    bl Phaze_C_OFF
    b _end

_case4:
    bl Phaze_A_ZZ
    bl Phaze_B_OFF
    bl Phaze_C_ON
    b _end

_case5:
    bl Phaze_A_ON
    bl Phaze_B_OFF
    bl Phaze_C_ZZ
    b _end

_case6:
    bl Phaze_A_OFF
    bl Phaze_B_ZZ
    bl Phaze_C_ON
    b _end

_zero_phases:
    bl Phaze_A_ZZ
    bl Phaze_B_ZZ
    bl Phaze_C_ZZ

_end:
    pop {r4-r7, pc}            // восстановление регистров и возврат

*/
