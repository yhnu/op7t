MODULE_NAME := rwProcMem33
RESMAN_CORE_OBJS:=sys.o
RESMAN_GLUE_OBJS:=phy_mem.o remote_proc_maps.o  remote_proc_cmdline.o remote_proc_rss.o
ifneq ($(KERNELRELEASE),)    
	$(MODULE_NAME)-objs:=$(RESMAN_GLUE_OBJS) $(RESMAN_CORE_OBJS)
	obj-m := rwProcMem33.o
else
	KDIR := /share/op7t/blu7t/op7-r70/out
all:
	make -C $(KDIR) REAL_CC=$(GITHUB_WORKSPACE)/buildtool/toolchains/llvm-Snapdragon_LLVM_for_Android_8.0/prebuilt/linux-x86_64/bin/clang \
	CROSS_COMPILE=/share/op7t/buildtool/aarch64-linux-android-4.9-uber-master/bin/aarch64-linux-android- CLANG_TRIPLE=aarch64-linux-gnu- \
	ARCH=arm64 M=$(PWD) modules CONFIG_MODULE_UNLOAD=y CONFIG_RETPOLINE=y
	make -C $(KDIR) ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- M=$(PWD) modules 

clean2:    
	rm -f *.ko *.o *.mod.o *.mod.c *.symvers *.order

clean:
	rm -rf *.o *.order *.symvers *.ko *.mod* .*.cmd .*.*.cmd .*.*.*.cmd
	@rm -fr .tmp_versions Module.symvers modules.order
endif    
