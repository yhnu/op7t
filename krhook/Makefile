# Code wrire 	: yhnu
# code date 	: 2021年9月10日 09:46:05
# e-mail	: buutuud@gmail.com
#
# THis Makefile is a demo only for ARM-architecture
#

MODULE_NAME := krhook
# MODULE_OBJS:=lkm_android.o
# $(MODULE_NAME)-objs:=$(MODULE_OBJS)
# 
ifneq ($(KERNELRELEASE),)
	obj-m := krhook.o

else
	CROSS_COMPILE := /share/op7t/buildtool/aarch64-linux-android-4.9-uber-master/bin/aarch64-linux-android-
	PWD	:=$(shell pwd)
	KDIR := $(GITHUB_WORKSPACE)/blu7t/op7-r70/out

modules:
	make -C $(KDIR) REAL_CC=$(GITHUB_WORKSPACE)/buildtool/toolchains/llvm-Snapdragon_LLVM_for_Android_8.0/prebuilt/linux-x86_64/bin/clang \
	CROSS_COMPILE=$(GITHUB_WORKSPACE)/buildtool/aarch64-linux-android-4.9-uber-master/bin/aarch64-linux-android- \
	CLANG_TRIPLE=aarch64-linux-gnu- ARCH=arm64 M=$(PWD) modules CONFIG_MODULE_UNLOAD=y CONFIG_RETPOLINE=y

clean:
	rm -rf *.*.tmp *.*.d *.o *.order *.symvers *.ko *.mod* .*.cmd .*.*.cmd .*.*.*.cmd	
endif

one7:
	adb kill-server
	adb connect 10.11.246.131

push:
	adb push krhook.ko /data/local/tmp/
	