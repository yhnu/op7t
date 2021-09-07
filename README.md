# op7t

oneplus 7t 自定义内核

## 手机环境

a. oneplus 7t

b. 对应Android版本为Hydrogen OS 10.0.7.HD65

<img src="https://cdn.jsdelivr.net/gh/yhnu/PicBed/images20210907091421.png" height="400" />

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

3. 编译依赖

链接: https://pan.baidu.com/s/1RQaPSjLPNX99XBVCCAcoZA 提取码: bbki 复制这段内容后打开百度网盘手机App，操作更方便哦

```shell
cd /share
git clone https://gitee.com/yhnu/op7t.git //这里不行,只能网盘存储了
cd /share/op7t/buildtool
just c
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

https://mp.weixin.qq.com/s?__biz=Mzg5MzU3NzkxOQ==&mid=2247483992&idx=4&sn=1137e15288c238668fb39462e295d82c&chksm=c02dfd88f75a749e38c772d45d620a615d77d21b2f3c729ecd767a13a4c674204fe6120beea2&scene=178&cur_album_id=1799542483832324102#rd


