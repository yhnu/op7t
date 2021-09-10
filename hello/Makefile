# Code wrire 	: yhnu
# code date 	: 2021年9月10日 09:46:05
# e-mail	: buutuud@gmail.com
#
# THis Makefile is a demo only for ARM-architecture
#

MODULE_NAME := hello

ifneq ($(KERNELRELEASE),)

	obj-m := hello.o

else
	CROSS_COMPILE := /share/op7t/buildtool/aarch64-linux-android-4.9-uber-master/bin/aarch64-linux-android-
	#CC = CROSS_COMPILE	
	#KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD	:=$(shell pwd)
	KDIR := /share/op7t/blu7t/op7-r70/out

modules:
	make -C $(KDIR) REAL_CC=$(GITHUB_WORKSPACE)/buildtool/toolchains/llvm-Snapdragon_LLVM_for_Android_8.0/prebuilt/linux-x86_64/bin/clang CROSS_COMPILE=/share/op7t/buildtool/aarch64-linux-android-4.9-uber-master/bin/aarch64-linux-android- CLANG_TRIPLE=aarch64-linux-gnu- ARCH=arm64 M=$(PWD) modules CONFIG_MODULE_UNLOAD=y CONFIG_RETPOLINE=y

clean:
	rm -rf *.o *.order *.symvers *.ko *.mod* .*.cmd .*.*.cmd .*.*.*.cmd
	@rm -fr .tmp_versions Module.symvers modules.order
endif