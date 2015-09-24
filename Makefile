CC = xtensa-lx106-elf-gcc
CFLAGS = -DLWIP_RAW -DLWIP_OPEN_SRC -I. -I ~/esp-open-sdk/esp_iot_sdk_v1.3.0/include -I ~/esp-open-sdk/sdk/examples/driver_lib/include -mlongcalls
LDLIBS = -nostdlib -Wl,--start-group -lmain -lnet80211 -lwpa -llwip -lpp -lphy -Wl,--end-group -lgcc
LDFLAGS = -Teagle.app.v6.ld

taggy-0x00000.bin: taggy
	esptool.py elf2image $^

taggy: taggy.o ping.o

taggy.o: taggy.c

ping.o: ping.c

flash: taggy-0x00000.bin
	killall -9 minicom || true
	esptool.py -p /dev/ttyUSB0 -b 115200 write_flash 0 taggy-0x00000.bin 0x40000 taggy-0x40000.bin

clean:
	rm taggy *.bin *.o
