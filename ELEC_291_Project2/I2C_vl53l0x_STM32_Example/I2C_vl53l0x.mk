SHELL=cmd
CC=arm-none-eabi-gcc
AS=arm-none-eabi-as
LD=arm-none-eabi-ld
CCFLAGS=-mcpu=cortex-m0 -mthumb -g

# Search for the path of the right libraries.  Works only on Windows.
GCCPATH=$(subst \bin\arm-none-eabi-gcc.exe,\,$(shell where $(CC)))
LIBPATH1=$(subst \libgcc.a,,$(shell dir /s /b "$(GCCPATH)*libgcc.a" | find "v6-m"))
LIBPATH2=$(subst \libc_nano.a,,$(shell dir /s /b "$(GCCPATH)*libc_nano.a" | find "v6-m"))
LIBSPEC=-L"$(LIBPATH1)" -L"$(LIBPATH2)"

OBJS=main.o hardware.o tof_sensors.o ir_motor.o serial.o startup.o newlib_stubs.o

PORTN=$(shell type COMPORT.inc)

# For smaller hex file remove '-u _printf_float' below
main.elf : $(OBJS)
#	$(LD) $(OBJS) $(LIBSPEC) -Os -u _printf_float -nostdlib -lnosys -lgcc -T ../Common/LDscripts/stm32l051xx.ld --cref -Map main.map -o main.elf
	$(LD) $(OBJS) $(LIBSPEC) -Os -nostdlib -lnosys -lgcc -T ../Common/LDscripts/stm32l051xx.ld --cref -Map main.map -o main.elf
	arm-none-eabi-objcopy -O ihex main.elf main.hex
	@echo Success!

main.o: main.c hardware.h tof_sensors.h ir_motor.h
	$(CC) -c $(CCFLAGS) main.c -o main.o

hardware.o: hardware.c hardware.h
	$(CC) -c $(CCFLAGS) hardware.c -o hardware.o

tof_sensors.o: tof_sensors.c tof_sensors.h hardware.h
	$(CC) -c $(CCFLAGS) tof_sensors.c -o tof_sensors.o

ir_motor.o: ir_motor.c ir_motor.h hardware.h
	$(CC) -c $(CCFLAGS) ir_motor.c -o ir_motor.o

startup.o: ../Common/Source/startup.c
	$(CC) -c $(CCFLAGS) -DUSE_USART1 ../Common/Source/startup.c -o startup.o

serial.o: ../Common/Source/serial.c
	$(CC) -c $(CCFLAGS) ../Common/Source/serial.c -o serial.o

newlib_stubs.o: ../Common/Source/newlib_stubs.c
	$(CC) -c $(CCFLAGS) ../Common/Source/newlib_stubs.c -o newlib_stubs.o

clean:
	@del $(OBJS) 2>NUL
	@del main.elf main.hex main.map 2>NUL
	@del *.lst 2>NUL

Flash_Load:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	@echo ..\stm32flash\stm32flash -w main.hex -v -g 0x0 ^^>sflash.bat
	@..\stm32flash\BO230\BO230 -b >>sflash.bat
	@sflash.bat
	@echo cmd /c start putty.exe -sercfg 115200,8,n,1,N -serial ^^>sputty.bat
	@..\stm32flash\BO230\BO230 -r >>sputty.bat
	@sputty

putty:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	@echo cmd /c start putty.exe -sercfg 115200,8,n,1,N -serial ^^>sputty.bat
	@..\stm32flash\BO230\BO230 -r >>sputty.bat
	@sputty

Picture:
	@cmd /c start STM32_to_VL53L0x.jpg

explorer:
	@explorer .