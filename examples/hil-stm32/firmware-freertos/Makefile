# FreeRTOS 版韌體。跟 ../firmware 共用 regs.h / proto.h / calib.h / crc16 / 連結腳本。
#   docker run --rm --network none -v "$PWD/..":/w -w /w/firmware-freertos --user "$(id -u):$(id -g)" <arm 映像> make
CC      = arm-none-eabi-gcc
NM      = arm-none-eabi-nm
SIZE    = arm-none-eabi-size
K       = kernel
# ARM_CM4F port 要 hard-float(它會開 FPU、存 FP 暫存器);裸機版是 soft-float,兩者不能混連。
CFLAGS  = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O2 -g -Wall -Wextra \
          -ffreestanding -fno-builtin -nostdlib -ffunction-sections -fdata-sections \
          -I. -Ilibc-min -I../firmware -I$(K)/include -I$(K)/portable/GCC/ARM_CM4F
LDFLAGS = -T ../firmware/stm32f4.ld -nostdlib -Wl,--gc-sections -Wl,-Map=build/hilctl-rtos.map
SRCS    = startup-rtos.c main-rtos.c ../firmware/crc16.c \
          $(K)/tasks.c $(K)/queue.c $(K)/list.c $(K)/portable/GCC/ARM_CM4F/port.c $(K)/portable/MemMang/heap_4.c
OBJS    = $(patsubst %.c,build/%.o,$(notdir $(SRCS)))
# 只讓 .c 走搜尋路徑;通用 VPATH 會把 build/crc16.o 解析到 ../firmware/build/crc16.o(踩過)
vpath %.c . ../firmware $(K) $(K)/portable/GCC/ARM_CM4F $(K)/portable/MemMang

all: build/hilctl-rtos.elf build/hilctl-rtos.sym

# 標頭相依:calib.h 由 calib.json 重生時,沒有這一行 make 不會重編(2026-09-16 踩到:增益改了 RTOS 版沒跟上)
build/%.o: %.c ../firmware/regs.h ../firmware/proto.h ../firmware/calib.h ../firmware/crc16.h FreeRTOSConfig.h
	@mkdir -p build
	$(CC) $(CFLAGS) -c $< -o $@

build/hilctl-rtos.elf: $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -lgcc -o $@
	$(SIZE) $@

build/hilctl-rtos.sym: build/hilctl-rtos.elf
	$(NM) -n $< > $@

clean:
	rm -rf build

.PHONY: all clean
