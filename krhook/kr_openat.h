#ifndef KRHOOK_KR_OPENAT
#define KRHOOK_KR_OPENAT

typedef asmlinkage long (*TYPE_openat)(int dfd, const char __user *filename, int flags, umode_t mode);
static TYPE_openat old_openat;

long new_openat(int dfd, const char __user *filename, int flags, umode_t mode) {
    long ret = old_openat(dfd, filename, flags, mode);    
    // const int pid                = current->pid;
    const struct cred *m_cred = current_cred();
    // is user process && can get pid && pid filter
    // if(m_cred->uid.val > 10000 && !kstrtol(g_buf[minor], 0, &mypid) && (pid == mypid || mypid == -1))
    if ((m_cred->uid.val > 10000) && (myuid == m_cred->uid.val || myuid < 0)) {
        const int tgid               = current->tgid;
        char path_buf[MAX_PATH] = {0};
        int len                      = 0;
        int ppid                     = 0;
        if (current->parent != NULL) {
            ppid = current->parent->pid;
        }
        len = strncpy_from_user(path_buf, filename, MAX_PATH);

        if (!strstr(path_buf, "/cmdline") && !strstr(path_buf, "/sys/devices/system/cpu/online") && strncmp(path_buf, "/system", strlen("/system")) && strncmp(path_buf, "/apex", strlen("/apex"))) {
            struct stack_trace trace = {.nr_entries = 0, .skip = 0, .max_entries = MAX_STACK_TRACE_DEPTH};
            unsigned long *entries   = NULL;
            char *stack_buf          = NULL;

            entries = kzalloc(MAX_STACK_TRACE_DEPTH * sizeof(unsigned long), GFP_KERNEL);

            if (!entries)
                goto Exit0;

            trace.entries = entries;
            save_stack_trace_user(&trace);
            // 0xffffffffffffffff
            // #define STACK_BUF_SIZE MAX_STACK_TRACE_DEPTH * sizeof("0xffffffffffffffff  ")
            stack_buf = kzalloc(STACK_BUF_SIZE, GFP_KERNEL);
            if (!stack_buf)
                goto Exit1;

            // memset(stack_buf, 0, STACK_BUF_SIZE);
            // snprint_stack_trace(stack_buf, STACK_BUF_SIZE, &trace, 1);

            printk("[s]openat %s tgid=%d\n", path_buf, tgid);
            // print_map_list();
            // printk("[e]openat [%s] current->pid:[%d] ppid:[%d] uid:[%d] tgid:[%d] stack:%s\n", bufname, pid, ppid, m_cred->uid.val, tgid, stack_buf);
            kfree(stack_buf);
        Exit1:
            kfree(entries);            
        }
    }
Exit0:
    return ret;
}

#define KR_OPENAT_INIT()                                                            \
    old_openat                      = (TYPE_openat)sys_call_table_ptr[__NR_openat]; \
    sys_call_table_ptr[__NR_openat] = (TYPE_openat)new_openat;

#define KR_OPENAT_CLEAN()                                \
    if (sys_call_table_ptr[__NR_openat] == new_openat) { \
        sys_call_table_ptr[__NR_openat] = old_openat;    \
    }

#endif /* KRHOOK_KR_OPENAT */
