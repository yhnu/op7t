# 内核层偷天换日之hook openat进行文件重定向

## openat重定向文件读写的一些问题

前段时间, 卓玛星球的强哥, 提出了在内核层进行文件重定向的想法. 但是他告诉我这里面有很多限制.比如__user, const等. 但其实并没有提到对应的核心原因.

1. __user和const都是类型修饰符而已, 主要目标是进行编译器检查. 但是在运行时是没有这种检查,因此我们有N种方法进行处理

2. 文件重定向就是进行文件路径修改, 我们不能直接修改的原因.得从arm内存访问以及代码实现说起.

## GCC常用修饰符

![20211011125404](https://cdn.jsdelivr.net/gh/yhnu/PicBed/20211011125404.png)

## do_sys_open底层在做什么事情

* asmlinkage long sys_openat(int dfd, const char __user *filename, int flags, umode_t mode);
  * do_sys_open(int dfd, const char __user *filename, int flags, umode_t mode)
    * tmp = getname(filename);
      * getname_flags(filename, 0, NULL);
        * step1 检查是否可以重用, 如果可以复用直接返回
        * audit_reusename(filename);
          * 遍历current->audit_context->names_list
          * __audit_reusename(name) //fill out filename with info from existing entry
            * if (n->name->uptr == uptr) { //检查uptr地址是否一致,一致的话refcnt++
        * step2 embed the struct filename inside the names_cache             
        * len = strncpy_from_user(kname, filename, EMBEDDED_NAME_MAX); //这里面有一次filename的访问, 这里是EMBEDDED_NAME_MAX
        * step3 接近PATH_MAX,拆分后在插入
        * len = strncpy_from_user(kname, filename, PATH_MAX); //这里是EMBEDDED_NAME_MAX

上面列出所有访问__user filename的地方,到后面进入long strncpy_from_user(char *dst, const char __user *src, long count), 

## strncpy_from_user底层到底在做什么事情

* strncpy_from_user 前面主要是做一些合法性检查
   * do_strncpy_from_user(dst, src, count, max);
     * 判断字节对齐
     * 缺页处理
     * byte_at_a_time:
       * unsafe_get_user


最开始我想直接修改源码去掉__user检查限制, 后面发现我太天真了, 层层限制,层层都有__user检查.

不过还有个终极手段就是直接改掉__user修饰符, 思来想去还是没去做这个事情, 因为总感觉自己做的不对. 因此切换了思路, 我遇到的问题肯定别人也遇到过.


## 简单粗暴memcopy

A思路走不通, 那就走B思路呗. 我仅仅是想修改内存而已, 我都是有特权级别的人了, 啥不能搞呢? 先写个代码试试看

```c
// access userspace in kernel
do
{            
    mm_segment_t fs;
    fs =get_fs();
    set_fs(KERNEL_DS);
    if(strstr(pathname, "a.txt"))
    {
        if(access_ok(VERIFY_WRITE, pathname, len))
        {
            memcpy(pathname+len-5, "b.txt", 5); //受CONFIG_ARM64_PAN配置限制

            if(copy_to_user(pathname+len-5, "b.txt", 5)) {
                printk("[i] bingo magic\n");
            } else {
                printk("[err] copy to usr error. %p len=%d\n", pathname, len);
            }

            len = strncpy_from_user(bufname, pathname, 255);
            printk("[i] bufname =%s\n", bufname);

        }
        else {
            printk("[e] access not ok\n");
        }
    }            
    set_fs(fs);
} while (0);  
```

似乎结果并不太好, 内核层崩溃了. 为啥呢? 思来想去查了很多资料, 重要找到了答案, 内核层有一个配置开关CONFIG_ARM64_PAN

1. CONFIG_ARM64_PAN 配置选项的功能是阻止内核态直接访问用户地址空间

具体核心代码如下:

```c
// arch\arm64\include\asm\uaccess.h
#define __uaccess_disable(alt)						\
do {									\
	if (!uaccess_ttbr0_disable())					\
		asm(ALTERNATIVE("nop", SET_PSTATE_PAN(1), alt,		\
				CONFIG_ARM64_PAN));			\
} while (0)

#define __uaccess_enable(alt)						\
do {									\
	if (!uaccess_ttbr0_enable())					\
		asm(ALTERNATIVE("nop", SET_PSTATE_PAN(0), alt,		\
				CONFIG_ARM64_PAN));			\
} while (0)
```

## 修改后对应的风险(欢迎大佬提供思路)

因为我们直接更改了用户层的数据, 这里面就隐含了其他风险, 对应的用户层的路径名直接就变更了.用户层再拿着这个
数据进行处理, 就会遇到一些问题.

我后面仔细想了想还可以通过修改libc.so的方式, 但是很多防护系统都做了字段openat. 因此我们只能去手动修改对应的so了. 

欢迎各群里有理解filesystem系统的大佬提供下对应的意见和想法.

## 具体源码参考与实现

[https://github.com/yhnu/op7t](https://github.com/yhnu/op7t)


## 参考链接:

[https://github.com/yhnu/op7t/commit/ec30f893b42aba551c84b7036aaa71f012703f4a](https://github.com/yhnu/op7t/commit/ec30f893b42aba551c84b7036aaa71f012703f4a)

[https://developer.ibm.com/articles/l-kernel-memory-access/](https://developer.ibm.com/articles/l-kernel-memory-access/)

[http://www.wowotech.net/memory_management/454.html](http://www.wowotech.net/memory_management/454.html)

[https://android.googlesource.com/kernel/common/+/android-trusty-4.14/arch/arm64/lib/copy_template.S](https://android.googlesource.com/kernel/common/+/android-trusty-4.14/arch/arm64/lib/copy_template.S)