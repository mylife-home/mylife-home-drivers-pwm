# http://lxr.free-electrons.com/source/Documentation/kbuild/modules.txt
# https://github.com/ant9000/soft_pwm/blob/master/soft_pwm.c
# https://github.com/sarfata/pi-blaster/blob/master/pi-blaster.c

obj-m = dma_pwm.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

modules_install:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules_install

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
