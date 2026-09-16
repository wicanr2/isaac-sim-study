# 在 docker 裡建(arm-none-eabi-gcc 12.2):
#   docker run --rm --network none -v "$PWD":/w -w /w --user "$(id -u):$(id -g)" \
#     <arm 工具鏈映像> make
# 產物:build/hilctl.elf(給 Renode LoadELF)、build/hilctl.sym(給橋接查 SRAM 位址)
CC      = arm-none-eabi-gcc
NM      = arm-none-eabi-nm
SIZE    = arm-none-eabi-size
CFLAGS  = -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -O2 -g -Wall -Wextra \
          -ffreestanding -fno-builtin -nostdlib -ffunction-sections -fdata-sections
LDFLAGS = -T stm32f4.ld -nostdlib -Wl,--gc-sections -Wl,-Map=build/hilctl.map
SRCS    = startup.c crc16.c control.c main.c
OBJS    = $(SRCS:%.c=build/%.o)

all: build/hilctl.elf build/hilctl.sym

build/%.o: %.c regs.h proto.h calib.h crc16.h control.h | build
	$(CC) $(CFLAGS) -c $< -o $@

build/hilctl.elf: $(OBJS) stm32f4.ld
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -lgcc -o $@
	$(SIZE) $@

build/hilctl.sym: build/hilctl.elf
	$(NM) -n $< > $@

build:
	mkdir -p build

clean:
	rm -rf build

.PHONY: all clean
