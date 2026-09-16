/* 連結腳本:對應 Renode 內建 stm32f4.repl 的記憶體配置
 * (flash 2 MB @0x08000000、SRAM 256 KB @0x20000000)。
 * 真板子(如 F407:1 MB flash / 128 KB SRAM)也放得下,這支韌體不到 16 KB。 */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 2M
  SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}

ENTRY(Reset_Handler)
_estack = ORIGIN(SRAM) + LENGTH(SRAM);

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
  .text : {
    *(.text*)
    *(.rodata*)
    . = ALIGN(4);
    _etext = .;
  } > FLASH

  _sidata = LOADADDR(.data);
  .data : {
    . = ALIGN(4);
    _sdata = .;
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > SRAM AT > FLASH

  .bss : {
    . = ALIGN(4);
    _sbss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > SRAM

  /* 重置後不清:暖重置計數(IWDG 驗收靠它;RCC_CSR 的旗標 Renode 不設)。
   * NOLOAD:LoadELF 不寫、startup 不碰,SRAM 內容跨 reset 保留。 */
  .noinit (NOLOAD) : {
    . = ALIGN(4);
    *(.noinit*)
    . = ALIGN(4);
  } > SRAM

  /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) }
}
