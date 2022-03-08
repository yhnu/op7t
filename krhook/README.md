# krhook

系统调用hook

## 开发记录
2021年10月29日 09:52:25 hook了外挂常用的系统调用

## 使用说明

```shell
insmod krhook.so myuid=10221

# dmesg 可以查看对应log

OnePlus7T:/data/local/tmp # dmesg -w | grep kr_
[36676.309022] [20211029_09:50:59.555441]@7 [info]kr_faccessat /vendor/lib64/hw/gralloc.msmnile.so uid=10226 stack:0x7c7ed82088|0x7bdf26a1a4|0x7aa1eeb96c|0x7aa1ea624c|0x7c7deedafc|0x7c7dee5c44|0x7b8f1aead8|0xffffffffffffffff|
[36676.310265] [20211029_09:50:59.556684]@7 [info]kr_faccessat /vendor/lib64/hw/gralloc.msmnile.so uid=10226 stack:0x7c7ed82088|0x7bdf26a1a4|0x7aa1eeb96c|0x7aa1ea4128|0x7c7deedba0|0x7c7dee5c44|0x7b8f1aead8|0xffffffffffffffff|
[36676.311047] [20211029_09:50:59.557466]@7 [info]kr_faccessat /vendor/lib64/hw/gralloc.msmnile.so uid=10226 stack:0x7c7ed82088|0x7bdf26a1a4|0x7aa1eeb96c|0x7aa1ea624c|0x7c7deedafc|0x7c7dee5c44|0x7b8f1aead8|0xffffffffffffffff|
[36676.311526] [20211029_09:50:59.557945]@7 [info]kr_faccessat /vendor/lib64/hw/gralloc.msmnile.so uid=10226 stack:0x7c7ed82088|0x7bdf26a1a4|0x7aa1eeb96c|0x7aa1ea4128|0x7c7deedba0|0x7c7dee5c44|0x7b8f1aead8|0xffffffffffffffff|
[36678.603886] [20211029_09:51:01.850302]@5 [info]kr_faccessat /storage/emulated/0/Android/obb/com.DefaultCompany.krhook_unity3d uid=10226 stack:0x7c7ed82088|0x71ac88f4|0x7bf8d1d67d|0xb176fd7e141a1536|0xffffffffffffffff|
[36678.649791] [20211029_09:51:01.896209]@6 [info]kr_faccessat /data/user/0/com.DefaultCompany.krhook_unity3d/shared_prefs/com.DefaultCompany.krhook_unity3d.v2.playerprefs.xml uid=10226 stack:0x7c7ed82088|0x71ac88f4|0x7bf8d1d67d|0xb176fd7e141a1536|0xffffffffffffffff|
[36678.649901] [20211029_09:51:01.896320]@6 [info]kr_faccessat /data/user/0/com.DefaultCompany.krhook_unity3d/shared_prefs/com.DefaultCompany.krhook_unity3d.v2.playerprefs.xml uid=10226 stack:0x7c7ed82088|0x71ac88f4|0x7bf8d1d67d|0xb176fd7e141a1536|0xffffffffffffffff|
[36678.649921] [20211029_09:51:01.896340]@6 [info]kr_faccessat /data/user/0/com.DefaultCompany.krhook_unity3d/shared_prefs/com.DefaultCompany.krhook_unity3d.v2.playerprefs.xml.bak uid=10226 stack:0x7c7ed82088|0x71ac88f4|0x7bf8d1d67d|0xb176fd7e141a1536|0xffffffffffffffff|
```

## 详细使用说明

[https://github.com/yhnu/op7t/tree/dev/kr_offline](https://github.com/yhnu/op7t/tree/dev/kr_offline)

## 偷天换日之文件重定向

[https://blog.csdn.net/Rong_Toa/article/details/86585086](https://blog.csdn.net/Rong_Toa/article/details/86585086)

[https://android.googlesource.com/kernel/common/+/android-trusty-4.14/arch/arm64/lib/copy_to_user.S](https://android.googlesource.com/kernel/common/+/android-trusty-4.14/arch/arm64/lib/copy_to_user.S)

[http://www.wowotech.net/memory_management/454.html](http://www.wowotech.net/memory_management/454.html)


```c
do
{            
    mm_segment_t fs;
    fs =get_fs();
    set_fs(KERNEL_DS);
    if(strstr(bufname, "a.txt"))
    {
        if(access_ok(VERIFY_WRITE, pathname, len))
        {
            if(copy_to_user(pathname+len-5, "b.txt", 5)) { //Internal error: Accessing user space memory with fs=KERNEL_DS: 9600004f [#1] PREEMPT SMP
                printk("[i] bingo magic\n");
            } else {
                printk("[e] copy to usr error. %p\n", pathname);
            }                    
        }
        else {
            printk("[e] access not ok\n");
        }
    }            
    set_fs(fs);
} while (0);

```

```shell
4,466251,1099988608,-;[20211009_16:11:06.256577]@2 [e]openat [/storage/emulated/0/Android/data/com.DefaultCompany.krhook_unity3d/files/a.txt] current->pid:[7671] ppid:[758] uid:[10226] tgid:[7636] stack:0x753214c388|0x742b412c18|0x742b434568|0x742be87054|0x742be7a930|0x742be78a6c|0x742be7493c|0x742be748b0|0x742bea7918|0x742b998fd8|0x742b9a5290|0x742b9a861c|0x742b7f68b0|0x742b7f6980|0x742b2f6750|0x742b7e10b8|0x742baf6d38|0x742bcce3fc|0x742b2f9ab0|0x742b7f0388|0x742b7ee83c|0x742b7ee0cc|0x742b25884c|0x742b7df7e4|0x742b274230|0x742b4a8eac|0x742b40bf04|0x74410c51d0|0xffffffffffffffff|
4,466252,1099988617,-;[20211009_16:11:06.256587]@2 [e] copy to usr error. 0000000064a1fc40
```
