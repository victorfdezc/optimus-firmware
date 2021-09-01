# optimus-firmware
Firmware for Optimus (Not so Prime) rover based on STM32F407ZGT6 microcontroller. In particular, the development board used is the STM32-E407 by Olimex.

# Ethernet Configuration for Olimex
As stated in Olimex schematics, the PHY is setted up as follows:
 * Mode: all capable (10/100 base)
 * Auto-negotiation enabled
 * RMII configuration
 * SMI (PHY) address: 0x00