#ifndef KRHOOK_KR_COMMON
#define KRHOOK_KR_COMMON

// 0xffffffffffffffff
#define STACK_BUF_SIZE MAX_STACK_TRACE_DEPTH * sizeof("0xffffffffffffffff  ")

int snprint_stack_trace(char *buf, size_t size, struct stack_trace *trace, int spaces) {
    int i;
    int generated;
    int total = 0;

    if (WARN_ON(!trace->entries))
        return 0;

    for (i = 0; i < trace->nr_entries; i++) {
        generated = snprintf(buf, size, "%pS|", (void *)trace->entries[i]);

        total += generated;

        /* Assume that generated isn't a negative number */
        if (generated >= size) {
            buf += size;
            size = 0;
        } else {
            buf += generated;
            size -= generated;
        }
    }

    return total;
}

static int print_map_list() {
    struct mm_struct *mm;
    struct vm_area_struct *vma;
    char path_buf[PATH_MAX];
    // DEFINE_HASHTABLE(htable, 3);
    // hash_init(htable);
    mm = get_task_mm(current);
    if (!mm) {
        return -2;
    }

    down_read(&mm->mmap_sem);
    for (vma = mm->mmap; vma; vma = vma->vm_next) {
        unsigned long start, end;
        unsigned char flags[5] = {0};

        start    = vma->vm_start;
        end      = vma->vm_end;
        flags[0] = vma->vm_flags & VM_READ ? 'r' : '-';
        flags[1] = vma->vm_flags & VM_WRITE ? 'w' : '-';
        flags[2] = vma->vm_flags & VM_EXEC ? 'x' : '-';
        flags[3] = vma->vm_flags & VM_SHARED ? 's' : 'p';

        if (vma->vm_file) {
            char *path;
            int len;
            memset(path_buf, 0, sizeof(path_buf));
            path = d_path(&vma->vm_file->f_path, path_buf, sizeof(path_buf));
            len  = strlen(path);
            if (path > 0 && !strncmp(path + len - 3, ".so", strlen(".so"))) {  // endswith .so
                unsigned long offset;
                offset = vma->vm_pgoff << PAGE_SHIFT;  // https://bbs.csdn.net/topics/390674592
                printk("[m]%pS-%pS %#010x %s %s\n", start, end, offset, flags, path);

            } else {
                printk("[m]%pS-%pS %s %s\n", start, end, flags, "");
            }
        }
    }  // end for

    up_read(&mm->mmap_sem);
    mmput(mm);

    return 0;
}

#define PRINT_STACK(tag, format, args...)                                   \
  do {                                                                      \
    struct stack_trace trace = {                                            \
        .nr_entries = 0, .skip = 0, .max_entries = MAX_STACK_TRACE_DEPTH};  \
    unsigned long* entries = NULL;                                          \
    char* stack_buf = NULL;                                                 \
    entries =                                                               \
        kzalloc(MAX_STACK_TRACE_DEPTH * sizeof(unsigned long), GFP_KERNEL); \
                                                                            \
    trace.entries = entries;                                                \
    save_stack_trace_user(&trace);                                          \
                                                                            \
    stack_buf = kzalloc(STACK_BUF_SIZE, GFP_KERNEL);                        \
    snprint_stack_trace(stack_buf, STACK_BUF_SIZE, &trace, 1);              \
    printk(tag format " stack:%s\n", ##args, stack_buf);                         \
    kfree(stack_buf);                                                       \
    kfree(entries);                                                         \
  } while (0);

#endif /* KRHOOK_KR_COMMON */
