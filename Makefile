obj-m := MFRC522.o
#CFLAGS_MFRC522.o := -DDEBUG

KDIR =  /home/elinux/linux-4.4.96

PWD := $(shell pwd)

default:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) SUBDIRS=$(PWD) modules
	arm-linux-gcc RFIDDUMP.c -o dump
clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean
	rm -rf dump 

#make ARCH=arm CROSS_COMPILE=arm-linux-
