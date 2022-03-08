# op7t

oneplus 7t 自定义内核

## 手机环境

a. oneplus 7t

b. 对应Android版本为Hydrogen OS 10.0.7.HD65

c. 对应版本全量包

```shell
OnePlus7THydrogen_14.H.09_OTA_009_all_2001030048_d935aae55ac_1007.zip //一加手机论坛下载
```

## 如何替换内核

[https://github.com/yhnu/note/tree/master/kernel/01_7t_install.md](https://github.com/yhnu/note/tree/master/kernel/01_7t_install.md)

## Linux编译环境

1. 操作系统ubuntu20

```shell
➜  /share cat /etc/lsb-release
DISTRIB_ID=Ubuntu
DISTRIB_RELEASE=20.04
DISTRIB_CODENAME=focal
DISTRIB_DESCRIPTION="Ubuntu 20.04 LTS"
```

2. 依赖安装

```shell
sudo apt-get install git-core gnupg flex bison build-essential zip curl zlib1g-dev gcc-multilib g++-multilib libc6-dev-i386 libncurses5-dev lib32ncurses5-dev x11proto-core-dev libx11-dev lib32z1-dev libgl1-mesa-dev libxml2-utils xsltproc unzip fontconfig

sudo apt-get install libssl-dev
sudo apt-get install dos2unix
sudo apt-get install libncurses5
```

3. 编译依赖

```shell
请参考GitHub CI
```

4. 编译源码

为了简单方便及稳定,我们基于第三方内核进行修改, [https://github.com/engstk/op7/tree/r70](https://github.com/engstk/op7/tree/r70)

```shell
cd /share/op7t/blu7t
just c
source env.config
source proxy.config
just make
just j16
```

5. 打包image

```shell
just image
```

## boot.img提取

```shell
curl -L -O https://github.com/yhnu/op7t/releases/download/v1.0/payload_dumper-win64.zip
curl -L -O https://otafsc.h2os.com/patch/CHN/OnePlus7THydrogen/OnePlus7THydrogen_14.H.09_009_2001030048/OnePlus7THydrogen_14.H.09_OTA_009_all_2001030048_d935aae55ac.zip
unzip -q OnePlus7THydrogen_14.H.09_009_2001030048/OnePlus7THydrogen_14.H.09_OTA_009_all_2001030048_d935aae55ac.zip
# 然后参考payload_dumper的使用说明即可
```

## 拯救OnePlus7t

开发的路上难免磕磕碰碰, 手机就启动不了, 做好防身技能

```shell
curl -L -O https://otafsc.h2os.com/patch/CHN/OnePlus7THydrogen/OnePlus7THydrogen_14.H.09_009_2001030048/OnePlus7THydrogen_14.H.09_OTA_009_all_2001030048_d935aae55ac.zip
curl -L -O https://github.com/yhnu/op7t/releases/download/v1.0/recovery-oneplus7t-3.4.2-10.0-b26.img
fastboot set_active a #is a or b
fastboot erase recovery
fastboot.exe flash recovery recovery-oneplus7t-3.4.2-10.0-b26.img 
fastboot.exe reboot recovery
adb sideload F:\F2021-07\one7t_kernel\OnePlus7THydrogen_14.H.09_OTA_009_all_2001030048_d935aae55ac.zip 
```

## 开发记录

2021年9月7日 14:33:03 

a. 修改内核源码绕过反调试检测

参考文章:

[修改内核源码绕过反调试检测](https://mp.weixin.qq.com/s?__biz=Mzg5MzU3NzkxOQ==&mid=2247483992&idx=4&sn=1137e15288c238668fb39462e295d82c&chksm=c02dfd88f75a749e38c772d45d620a615d77d21b2f3c729ecd767a13a4c674204fe6120beea2&scene=178&cur_album_id=1799542483832324102#rd)

[相关修改_对应分支](https://github.com/yhnu/op7t/anti)

2021年9月10日 08:56:45

a. 编写hellomod模块

```Makefile
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
```
b. 注意:

因为内核编译使用的是Clang编译, 因此对应module的编译也需要使用Clang编译

[module的加载过程](https://www.cnblogs.com/sky-heaven/p/5569240.html)

[clang交叉编译](https://blog.csdn.net/qq_23599965/article/details/90901235)

```shell
triple 的一般格式为<arch><sub>-<vendor>-<sys>-<abi>，其中：

arch = x86_64、i386、arm、thumb、mips等。
sub = v5, v6m, v7a, v7m等。
vendor = pc, apple, nvidia, ibm,等。
sys = none, linux, win32, darwin, cuda等。
abi = eabi, gnu, android, macho, elf等。
```

2021年9月10日 16:33:14

a. linux设备驱动开发学习经典教程

[https://lwn.net/Kernel/LDD3/](https://lwn.net/Kernel/LDD3/)

2021年9月11日 21:14:44

## 查看KernelLog

```shell
adb logcat -b kernel,default 
```

2021年9月17日 09:01:09

## krhook模块开发

https://github.com/yhnu/op7t/tree/dev/krhook

参考链接:

https://bbs.pediy.com/thread-267004.htm

现在Android 手机大都使用了 MSM 平台 和 kernel， 高通下面的一个patch 引入了 kernel 代码段内存RO 属性. 因此需要做一些修改

2021年9月17日 14:28:50

友人提醒可以直接通过下面的编译器进行编译,不用改代码Makefile

https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-a/downloads

![3cb1209765988e17c4de9b078a07eea](https://cdn.jsdelivr.net/gh/yhnu/PicBed/images3cb1209765988e17c4de9b078a07eea.png)

2021年9月17日 16:28:58

1. /sys/fs/pstore 内核崩溃日志信息存放的位置

2. linux使用lwp机制, 导致task_struct的pid在线程中是线程id,需要使用tgid,或者使用uid进行识别

http://www.opensourceforu.com/2011/08/light-weight-processes-dissecting-linux-threads/

### 2021年9月30日 11:06:14

special thanks: https://github.com/abcz316/rwProcMem33



3. linux进程管理

https://blog.csdn.net/o_alpha/article/details/95305837

https://blog.csdn.net/qq_34696349/article/details/118281098

https://www.cnblogs.com/crybaby/p/13052993.html


4. tail of dmesg

```shell
while true; do dmesg -c ; sleep 1 ; done
# huo zhe
adb logcat -c && adb logcat -b kernel,default |rg 8641

# /proc/kmsg and /dev/kmsg provide log data in a non-RFC-5424 form.
# https://blog.csdn.net/weixin_39834281/article/details/116657611
cat /dev/kmsg
```

5. syscall相关

https://blog.csdn.net/weixin_42915431/article/details/115289115

6. kernel stack相关

https://wenboshen.org/posts/2015-12-18-kernel-stack.html

![20210917172958](https://cdn.jsdelivr.net/gh/yhnu/PicBed/20210917172958.png)

## 添加UserSpace Stack Walk

```shell
[  436.292917] [20210918_10:22:39.895518]@4 stack_size:[12]
[  436.292919] stack:[<0000000000000000>] 0x7b19886388
[  436.292924] [20210918_10:22:39.895525]@4 stack:[<0000000000000000>] 0x7b19893d20
[  436.292932] [20210918_10:22:39.895533]@4 stack:[<0000000000000000>] 0x7b1762930c
[  436.292937] [20210918_10:22:39.895538]@4 stack:[<0000000000000000>] 0x7b176293fc
[  436.292941] [20210918_10:22:39.895542]@4 stack:[<0000000000000000>] 0x7b175f7ae0
[  436.292945] [20210918_10:22:39.895546]@4 stack:[<0000000000000000>] 0x7b175f6ca4
[  436.292949] [20210918_10:22:39.895550]@4 stack:[<0000000000000000>] 0x7b1a69758c
[  436.292954] [20210918_10:22:39.895555]@4 stack:[<0000000000000000>] 0x7b17743c60
[  436.292958] [20210918_10:22:39.895559]@4 stack:[<0000000000000000>] 0x7b1989bb74
[  436.292962] [20210918_10:22:39.895563]@4 stack:[<0000000000000000>] 0x7b19839eb0
[  436.292967] [20210918_10:22:39.895568]@4 stack:[<0000000000000000>] 0xffffffffffffffff
```

2021年9月28日 13:24:47

## run-as命令使用说明

在某个APP的权限下运行某些命令, 方便确认对应的权限

https://stackoverflow.com/questions/26954161/what-are-the-rwxp-sections-in-proc-pid-smaps-linux

http://books.gigatux.nl/mirror/kerneldevelopment/0672327201/ch14lev1sec2.html

rwxps对应的

    r = read
    w = write
    x = execute
    s = shared
    p = private (copy on write)

```shell
# http://adbcommand.com/articles/%E5%B8%B8%E7%94%A8adb%20shell%E5%91%BD%E4%BB%A4%EF%BC%9Arun-as%E5%92%8Cexec-out
OnePlus7T:/ $ run-as com.DefaultCompany.krhook_unity3d
OnePlus7T:/data/data/com.DefaultCompany.krhook_unity3d $ ls
cache code_cache files shared_prefs
```

## printk关于格式化说明

    需要说明的一个地方是，通过函数的地址来打印函数名是通过格式控制符%pS来打印的：

    printk("[<%p>] %pS\n", (void *) ip,(void *) ip);

    在内核代码树的lib/vsprintf.c中的pointer函数中，说明了printk中的%pS的意思：

    case 'S':

    return symbol_string(buf, end, ptr, spec, *fmt);

    即'S'表示打印符号名，而这个符号名是kallsyms里获取的。

    可以看一下kernel/kallsyms.c中的kallsyms_lookup()函数，它负责通过地址找到函数名，分为两部分：

    1. 如果地址在编译内核时得到的地址范围内，就查找kallsyms_names数组来获得函数名。
    2. 如果这个地址是某个内核模块中的函数，则在模块加载后的地址表中查找。
    kallsyms_lookup()最终返回字符串“函数名+offset/size[mod]”，交给printk打印。

## 防护系统

    https://github.com/darvincisec/AntiDebugandMemoryDump
    1. 实现了自己的syscall
    2. 实现了常用的libc函数
    3. 一些常规的检查手段
## 平时一些分析相关的笔记
    
   [https://github.com/yhnu/note](https://github.com/yhnu/note)
