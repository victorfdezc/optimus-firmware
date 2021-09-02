# optimus-firmware
Firmware for Optimus (Not so Prime) rover based on STM32F407ZGT6 microcontroller. In particular, the development board used is the STM32-E407 by Olimex.

## Ethernet Configuration for Olimex
As stated in Olimex schematics, the PHY is setted up as follows:
 * Mode: all capable (10/100 base)
 * Auto-negotiation enabled
 * RMII configuration
 * SMI (PHY) address: 0x00

## FreeRTOS on ARM-Cortex
"When FreeRTOS is used it is strongly recommended to use a HAL time base source other than the SysTick".
SysTick is a 24-bit system timer integrated in ARM-Cortex processors and it is normally used to provide OS/RTOS clock for context switching or timing services. By default, the STM32 HAL is built around it. However, HAL-timebase related functions are defined as weak so that they can be overloaded to use another hardware timebase source. This is strongly recommended when the application uses an RTOS, since this middleware has full control on the SysTick configuration (tick and priority) and most RTOSes force the SysTick priority to be the lowest (so do not change the SysTick configuration). Using the SysTick remains acceptable if the application does not perform any call to HAL-timebase services (dead lock issue).
To change the HAL timebase source, go to the SYS peripheral and select a clock among the available clock sources (TIM1, TIM2...).