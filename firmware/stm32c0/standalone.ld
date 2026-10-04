/* STM32C092KC application without the CAN bootloader (the carrier's IO MCU, flashed through
   ST's ROM bootloader): the whole flash. The top 16 bytes of RAM hold the boot flag. */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 256K
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 30K - 16
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x800;

INCLUDE sections.ld
