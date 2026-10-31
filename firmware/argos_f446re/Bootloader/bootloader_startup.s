  .syntax unified
  .cpu cortex-m4
  .thumb

  .global g_pfnVectors
  .global Reset_Handler

  .section .isr_vector,"a",%progbits
g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word Default_Handler /* NMI */
  .word Default_Handler /* HardFault */
  .word Default_Handler /* MemManage */
  .word Default_Handler /* BusFault */
  .word Default_Handler /* UsageFault */
  .word 0
  .word 0
  .word 0
  .word 0
  .word Default_Handler /* SVC */
  .word Default_Handler /* DebugMon */
  .word 0
  .word Default_Handler /* PendSV */
  .word SysTick_Handler
  .rept 96
  .word Default_Handler
  .endr

  .section .text.Reset_Handler,"ax",%progbits
  .type Reset_Handler,%function
Reset_Handler:
  ldr sp, =_estack
  ldr r0, =_sidata
  ldr r1, =_sdata
  ldr r2, =_edata
1:
  cmp r1, r2
  bcc 2f
  b 3f
2:
  ldr r3, [r0], #4
  str r3, [r1], #4
  b 1b
3:
  ldr r1, =_sbss
  ldr r2, =_ebss
  movs r3, #0
4:
  cmp r1, r2
  bcc 5f
  b 6f
5:
  str r3, [r1], #4
  b 4b
6:
  bl main
7:
  b 7b
  .size Reset_Handler, .-Reset_Handler

  .section .text.Default_Handler,"ax",%progbits
  .type Default_Handler,%function
Default_Handler:
  b Default_Handler
  .size Default_Handler, .-Default_Handler
