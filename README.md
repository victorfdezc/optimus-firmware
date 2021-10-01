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

## FreeRTOS and STM32CubeMX
FreeRTOS will not be added with STM32CubeMX, instead will be added by [source](https://github.com/FreeRTOS/FreeRTOS-Kernel). This will avoid extra libraries added by STM32CubeMX like CMSIS OS, which won't be used because the lack of documentation and the extra overkilling layer. Besides, this will allow easier code updates in case FreeRTOS is changed.
Since Olimex-E407 has an ARM Cortex M4, the port used is [ARM_CM4F](https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/8de8a9da1aa9b036812a72fdcd7cbdefc2789365/portable/GCC/ARM_CM4F).

## FreeRTOS TCP/IP or LwIP
It seems that LwIP doen't work well in STM32 and it's not hardware realted (see [1](https://community.st.com/s/question/0D50X0000BOtfhnSQB/how-to-make-ethernet-and-lwip-working-on-stm32) and [2](https://community.st.com/s/question/0D50X0000AhNBoWSQW/actually-working-stm32-ethernet-and-lwip-demonstration-firmware)). On the other hand, FreeRTOS has a TCP/IP implementation optimized for FreeRTOS and it's thread safe (some of its advanteges are explained [here](https://www.freertos.org/FreeRTOS_Support_Forum_Archive/November_2014/freertos_FreeRTOS_TCP_IP_stack_vs_lwIP_8324ceabj.html)). It will be studied the possibility of porting FreeRTOS TCP/IP to the Olimex board, instead of struggling with possible multiple problems which can appear by using LwIP.
