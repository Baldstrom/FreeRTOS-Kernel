# *
# * FreeRTOS Kernel <DEVELOPMENT BRANCH>
# * Copyright (C) 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# *
# * SPDX-License-Identifier: MIT
# *
# * Permission is hereby granted, free of charge, to any person obtaining a copy of
# * this software and associated documentation files (the "Software"), to deal in
# * the Software without restriction, including without limitation the rights to
# * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# * the Software, and to permit persons to whom the Software is furnished to do so,
# * subject to the following conditions:
# *
# * The above copyright notice and this permission notice shall be included in all
# * copies or substantial portions of the Software.
# *
# * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# *
# * https://www.FreeRTOS.org
# * https://github.com/FreeRTOS
# *
# */

        .section .text
        .syntax unified
        .cpu cortex-r4
        .arm
        .extern vTaskSwitchContext
        .extern xTaskIncrementTick
        .extern ulTaskHasFPUContext
        .extern pxCurrentTCB

        .global vPortStartFirstTask
        .global vPortYieldProcessor
        .global vPortYeildWithinAPI
        .global vPortPreemptiveTick

        #ifdef (__TI_VFP_SUPPORT__)
        .global vPortInitialiseFPSCR
        #endif

        .global getCPSR

# -----------------------------------------------------------
#
# Save Task Context
#
.macro          portSAVE_CONTEXT 
        DSB

        # Push R0 as we are going to use it
        STMDB   SP!, {R0}

        # Set R0 to point to the task stack pointer.
        STMDB   SP,{SP}^
        SUB SP, SP, #4
        LDMIA   SP!,{R0}

        # Push the return address onto the stack.
        STMDB   R0!, {LR}

        # Now LR has been saved, it can be used instead of R0.
        MOV LR, R0

        # Pop R0 so it can be saved onto the task stack.
        LDMIA   SP!, {R0}

        # Push all the system mode registers onto the task stack.
        STMDB   LR,{R0-LR}^
        SUB LR, LR, #60

        # Push the SPSR onto the task stack.
        MRS R0, SPSR
        STMDB   LR!, {R0}

    #ifdef (__TI_VFP_SUPPORT__)
        # Determine if the task maintains an FPU context.
        LDR R0, ulFPUContextConst
        LDR R0, [R0]

        # Test the flag
        CMP     R0, #0

        # If the task is not using a floating point context then skip the
        # saving of the FPU registers.
        BEQ     $+16
        FSTMDBD LR!, {D0-D15}
        FMRX    R1,  FPSCR
        STMFD   LR!, {R1}

        # Save the flag
        STMDB   LR!, {R0}
    #endif

        # Store the new top of stack for the task.
        LDR R0, pxCurrentTCBConst
        LDR R0, [R0]
        STR LR, [R0]

        .endm

# -----------------------------------------------------------
#
# Restore Task Context
#
.macro          portRESTORE_CONTEXT
        LDR     R0, pxCurrentTCBConst
        LDR     R0, [R0]
        LDR     LR, [R0]

    #ifdef (__TI_VFP_SUPPORT__)
        # The floating point context flag is the first thing on the stack.
        LDR     R0, ulFPUContextConst
        LDMFD   LR!, {R1}
        STR     R1, [R0]

        # Test the flag
        CMP     R1, #0

        # If the task is not using a floating point context then skip the
        # VFP register loads.
        BEQ     $+16

        # Restore the floating point context.
        LDMFD   LR!, {R0}
        FLDMIAD LR!, {D0-D15}
        FMXR    FPSCR, R0
    #endif

        # Get the SPSR from the stack.
        LDMFD   LR!, {R0}
        MSR     SPSR_csxf, R0

        # Restore all system mode registers for the task.
        LDMFD   LR, {R0-R14}^

        # Restore the return address.
        LDR     LR, [LR, #+60]

        # And return - correcting the offset in the LR to obtain the
        # correct address.
        SUBS    PC, LR, #4
        .endm

# -----------------------------------------------------------
# Start the first task by restoring its context.

        # .def vPortStartFirstTask
        .type vPortStartFirstTask, %function
vPortStartFirstTask:
        portRESTORE_CONTEXT

        .global vPortYield
vPortYield: .word swiPortYield

        .global swiPortYield
        .type swiPortYield, %function
swiPortYield:
        #  Restore stack and LR before calling vPortYieldProcessor
        ldmfd   sp!, {r11,r12,lr}

# -----------------------------------------------------------
# Yield to another task.

        # .def vPortYieldProcessor
        .type vPortStartFirstTask, %function

vPortYieldProcessor:
        # Within an IRQ ISR the link register has an offset from the true return
        # address.  SWI doesn't do this. Add the offset manually so the ISR
        # return code can be used.
        ADD     LR, LR, #4

        # First save the context of the current task.
        portSAVE_CONTEXT

        # Select the next task to execute. */
        BL      vTaskSwitchContext

        # Restore the context of the task selected to execute.
        portRESTORE_CONTEXT

# -----------------------------------------------------------
# Yield to another task from within the FreeRTOS API

        # .def vPortYeildWithinAPI
        .type vPortYeildWithinAPI, %function

vPortYeildWithinAPI:
        # Save the context of the current task.

        portSAVE_CONTEXT
        # Clear SSI flag.
        MOVW    R0, #0xFFF4
        MOVT    R0, #0xFFFF
        LDR     R0, [R0]

        # Select the next task to execute. */
        BL      vTaskSwitchContext

        # Restore the context of the task selected to execute.
        portRESTORE_CONTEXT

# -----------------------------------------------------------
# Preemptive Tick

        # .def vPortPreemptiveTick
        .type vPortPreemptiveTick, %function
vPortPreemptiveTick:

        # Save the context of the current task.
        portSAVE_CONTEXT

        # Clear interrupt flag
        MOVW    R0, #0xFC88
        MOVT    R0, #0xFFFF
        MOV     R1, #1
        STR     R1, [R0]

        # Increment the tick count, making any adjustments to the blocked lists
        # that may be necessary.
        BL      xTaskIncrementTick

        # Select the next task to execute.
        CMP R0, #0
        BLNE    vTaskSwitchContext

        # Restore the context of the task selected to execute.
        portRESTORE_CONTEXT

# ------------------------------------------------------------------------------


    #ifdef (__TI_VFP_SUPPORT__)

        # .def vPortInitialiseFPSCR
        .type vPortInitialiseFPSCR, %function

vPortInitialiseFPSCR:

        MOV     R0, #0
        FMXR    FPSCR, R0
        BX      LR

    #endif #__TI_VFP_SUPPORT__


        .type getCPSR, %function
getCPSR:
        MRS R0, CPSR

# ------------------------------------------------------------------------------

# -------------------------------------------------------------------------------
        .global ulPortCountLeadingZeros
        .type ulPortCountLeadingZeros, %function
ulPortCountLeadingZeros:
        CLZ     R0, R0
        BX      LR
# -------------------------------------------------------------------------------
#  SWI Handler, interface to Protected Mode Functions
        .global vPortSWI
        .type vPortSWI, %function
vPortSWI:
        stmfd   sp!, {r11,r12,lr}
        ldrb    r12, [lr, #-1]
        ldr     r14,  jumpTable
        ldr     r12, [r14, r12, lsl #2]
        blx     r12
        ldmfd   sp!, {r11,r12,pc}^

jumpTable:
        .word   swiPortYield                    ;#  0 - vPortYieldProcessor
        .word   swiRaisePrivilege               ;#  1 - Raise Priviledge
        .word   swiPortEnterCritical            ;#  2 - vPortEnterCritical
        .word   swiPortExitCritical             ;#  3 - vPortExitCritical
        .word   swiPortTaskUsesFPU              ;#  4 - vPortTaskUsesFPU
        .word   swiPortDisableInterrupts        ;#  5 - vPortDisableInterrupts
        .word   swiPortEnableInterrupts         ;#  6 - vPortEnableInterrupts

# -------------------------------------------------------------------------------
#  swiPortDisableInterrupts
        .global swiPortDisableInterrupts
        .type swiPortDisableInterrupts, %function
swiPortDisableInterrupts:
        mrs     r11, SPSR
        orr     r11, r11, #0x80
        msr     SPSR_c, r11
        bx      r14

# -------------------------------------------------------------------------------
#  swiPortEnableInterrupts
        .global swiPortEnableInterrupts
        .type swiPortEnableInterrupts, %function
swiPortEnableInterrupts:
        mrs     r11, SPSR
        bic     r11, r11, #0x80
        msr     SPSR_c, r11
        bx      r14

# -------------------------------------------------------------------------------
#  swiPortTaskUsesFPU
        .global swiPortTaskUsesFPU
        .type swiPortTaskUsesFPU, %function
swiPortTaskUsesFPU:
        ldr     r12, ulTaskHasFPUContextConst
        mov     r11, #1
        str     r11, [r12]
        mov     r11, #0
        fmxr    FPSCR, r11
        bx      r14

# -------------------------------------------------------------------------------
#  swiRaisePrivilege

#  Must return zero in R0 if caller was in user mode
        .global swiRaisePrivilege
        .type swiRaisePrivilege, %function
swiRaisePrivilege:
        mrs     r12, spsr
        ands    r0, r12, #0x0F      ;#  return value
        orreq   r12, r12, #0x1F
        msreq   spsr_c, r12
        bx      r14

# -------------------------------------------------------------------------------
#  swiPortEnterCritical
        .global swiPortEnterCritical
        .type swiPortEnterCritical, %function
swiPortEnterCritical:
        mrs     r11, SPSR
        orr     r11, r11, #0x80
        msr     SPSR_c, r11
        ldr     r11, ulCriticalNestingConst
        ldr     r12, [r11]
        add     r12, r12, #1
        str     r12, [r11]
        bx      r14

# -------------------------------------------------------------------------------
#  swiPortExitCritical
        .global swiPortExitCritical
        .type swiPortExitCritical, %function
swiPortExitCritical:
        ldr     r11, ulCriticalNestingConst
        ldr     r12, [r11]
        cmp     r12, #0
        bxeq    r14
        subs    r12, r12, #1
        str     r12, [r11]
        bxne    r14
        mrs     r11, SPSR
        bic     r11, r11, #0x80
        msr     SPSR_c, r11
        bx      r14

# -------------------------------------------------------------------------------
#  SetRegion

        .global prvMpuSetRegion
        .type prvMpuSetRegion, %function
#  void _mpuSetRegion(unsigned region, unsigned base, unsigned size, unsigned access)# 

prvMpuSetRegion:
        and   r0,  r0, #15                  ;#  select region
        mcr   p15, #0, r0, c6, c2, #0
        mcr   p15, #0, r1, c6, c1, #0       ;#  Base Address
        mcr   p15, #0, r3, c6, c1, #4       ;#  Access Attributes
        mcr   p15, #0, r2, c6, c1, #2       ;#  Size and Enable
        bx    lr

# -------------------------------------------------------------------------------
#  Enable Mpu

        .global prvMpuEnable
        .type prvMpuEnable, %function
prvMpuEnable:
        mrc   p15, #0, r0, c1, c0, #0
        orr   r0,  r0, #1
        dsb
        mcr   p15, #0, r0, c1, c0, #0
        isb
        bx    lr
# -------------------------------------------------------------------------------
#  Disable Mpu

        .global prvMpuDisable
        .type prvMpuDisable, %function
prvMpuDisable:
        mrc   p15, #0, r0, c1, c0, #0
        bic   r0,  r0, #1
        dsb
        mcr   p15, #0, r0, c1, c0, #0
        isb
        bx    lr
# -------------------------------------------------------------------------------

pxCurrentTCBConst: .word pxCurrentTCB
ulFPUContextConst: .word ulTaskHasFPUContext
ulCriticalNestingConst: .word ulCriticalNesting
ulTaskHasFPUContextConst: .word ulTaskHasFPUContext
