# Makefile for build iio_driver.c
obj-m += iio.o

iio-objs := iio_driver.o src/vl53l0x_api_calibration.o src/vl53l0x_api_core.o \
            src/vl53l0x_api_ranging.o src/vl53l0x_api_strings.o src/vl53l0x_api.o \
            src/vl53l0x_platform.o src/vl53l0x_i2c_platform.o src/vl53l0x_port_i2c.o

EXTRA_CFLAGS += -I$(PWD) -I$(PWD)/inc

all:
	make -C ~/linux-at91 M=$(PWD) modules

clean:
	make -C ~/linux-at91 M=$(PWD) clean
