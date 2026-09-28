/* STM32C092KC bootloader: the first 16 KB of flash. */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 16K
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 30K - 16
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size = 0;
_Min_Stack_Size = 0x400;

INCLUDE sections.ld
