# op7t

oneplus 7t 自定义内核

## 手机环境

a. oneplus 7t

b. 对应Android版本为Hydrogen OS 10.0.7.HD65

c. 对应版本全量包

```shell
OnePlus7THydrogen_14.H.09_OTA_009_all_2001030048_d935aae55ac_1007.zip //一加手机论坛下载
```

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

3. 编译源码

为了简单方便及稳定,我们基于第三方内核进行修改, [https://github.com/engstk/op7/tree/r70](https://github.com/engstk/op7/tree/r70)

```shell
cd /share/op7t/blu7t
just c
source env.config
source proxy.config
just make
just j16
```

4. 打包image

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
