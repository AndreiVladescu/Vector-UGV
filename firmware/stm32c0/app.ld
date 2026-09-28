/* STM32C092KC application: after the 16 KB bootloader, up to the two config pages (the
   last 4 KB). The top 16 bytes of RAM hold the bootloader flag, so the stack starts below them. */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x08004000, LENGTH = 236K
  CONFIG (r) : ORIGIN = 0x0803F000, LENGTH = 4K
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 30K - 16
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x800;

INCLUDE sections.ld
