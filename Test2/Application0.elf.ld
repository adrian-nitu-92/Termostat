
/*
!!! Primary linker script file generated automatically !!!
This file contains the input and output files names
It also includes the secondary/main linker script that is specified in the Ride options (default or custom)
*/

SEARCH_DIR(".")
SEARCH_DIR("C:\Users\Adrian\Desktop\Termostat\Test2")
SEARCH_DIR("C:\Program Files (x86)\Raisonance\Ride\Lib\ARM")
SEARCH_DIR("C:\Users\Adrian\Desktop\Termostat\Test2\lib\STM32F10x_StdPeriph_Driver\inc")
INPUT("C:\Users\Adrian\Desktop\Termostat\Test2\main.o")
INPUT("C:\Users\Adrian\Desktop\Termostat\Test2\stm32f10x_it.o")
INPUT("C:\Users\Adrian\Desktop\Termostat\Test2\sys_time.o")
INPUT("C:\Users\Adrian\Desktop\Termostat\Test2\stm32f10x_flash.o")
INPUT("C:\Users\Adrian\Desktop\Termostat\Test2\stm32f10x_gpio.o")
INPUT("C:\Users\Adrian\Desktop\Termostat\Test2\stm32f10x_rcc.o")
INPUT("C:\Users\Adrian\Desktop\Termostat\Test2\stm32f10x_tim.o")
INPUT("C:\Users\Adrian\Desktop\Termostat\Test2\misc.o")
INPUT("C:\Users\Adrian\Desktop\Termostat\Test2\stm32f10x_exti.o")
INPUT("C:\Users\Adrian\Desktop\Termostat\Test2\core_cm3.o")
INPUT("C:\Users\Adrian\Desktop\Termostat\Test2\system_stm32f10x.o")
INPUT("C:\Users\Adrian\Desktop\Termostat\Test2\startup_stm32f10x_md.o")
GROUP(
"std_sbrk_thumb.lib"
"smallprintf_thumb.a"
"libc.a"
"libm.a"

)
INCLUDE "C:\Users\Adrian\Desktop\Termostat\Test2\STM32F103_32K_20K_FLASH.ld"
OUTPUT("C:\Users\Adrian\Desktop\Termostat\Test2\Application0.elf")
