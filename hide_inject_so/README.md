
## 声明

学习交流, 请勿用于非法用途

## 防护系统如何检测你


续接上文[内核层偷天换日之hook openat进行文件重定向](https://bbs.pediy.com/thread-269730.htm)

我们在分析一些软件的时候. 防护系统通过扫描/proc/self/maps,发现一些异常的so, 那么就会进行上报. 我们有很多方式解决这种问题, 但是我还是希望能够了解内核层底层原理的同时, 能够非常方便的解决此问题.

## mmap文件映射需要注意的点

![20211014134043](https://cdn.jsdelivr.net/gh/yhnu/PicBed/20211014134043.png)

关于mmap的原理这里就不做详细的阐述, 这里面主要关注下文件映射, addr并不是完全相等映射, 只是相近而已. 通过官方的文档可以了解到.

![20211014134217](https://cdn.jsdelivr.net/gh/yhnu/PicBed/20211014134217.png)


## maps文件简单介绍

![20211014133558](https://cdn.jsdelivr.net/gh/yhnu/PicBed/20211014133558.png)

进程的每段地址空间由struct vm_area_struct 描述。如上所示的每一行对应一个vm_area_struct结构体。一个文件可以映射到内存中，vm_area_struct的vm_file保存了文件描述符，这种映射称为有名映射，反之则为匿名映射

1. 7857803000-785784b000 

	本段内存映射的虚拟地址空间范围, 对应vm_area_struct中的vm_start和vm_end

2. rw-p----权限 

	r-读，w-写 x-可执行 p-私有，对应vm_flags。

3. 00000000

	针对有名映射，指本段映射地址在文件中的偏移，对应vm_pgoff。对匿名映射而言，为vm_area_struct->vm_start

4. fd：00

	所映射的文件所属设备的设备号，对应vm_file->f_dentry->d_inode->i_sb->s_dev。匿名映射为0。其中fd为主设备号，00为次设备号

5. 00048000

	文件的索引节点号，对应vm_file->f_dentry->d_inode->i_ino，与ls –i显示的内容相符。匿名映射为0。

6. /data/local/tmp/xxxx.so

	所映射的文件名。对有名映射而言，是映射的文件名，对匿名映射来说，是此段内存在进程中的作用。[stack]表示本段内存作为栈来使用，[heap]作为堆来使用，其他情况则为无。

## cat /proc/self/map底层实现

在内核层fs系统实现了/proc/self/maps设备驱动, 其中核心实现在task_mm.c的show_map_vma函数中.
```c
static void show_vma_header_prefix2(struct seq_file *m,
				   unsigned long start, unsigned long end,
				   vm_flags_t flags, unsigned long long pgoff,
				   dev_t dev, unsigned long ino)
{
	seq_setwidth(m, 25 + sizeof(void *) * 6 - 1);
	seq_printf(m, "%08lx-%08lx %c%c%c%c %08llx %02x:%02x %lu ",
		   start,
		   end,
    //文件权限相关, 这里把可执行权限隐藏掉, 伪造成匿名映射.或者其他系统映射都可以
		   flags & VM_READ ? 'r' : '-',
		   flags & VM_WRITE ? 'w' : '-',
		   '-',//flags & VM_EXEC ? 'x' : '-',
		   flags & VM_MAYSHARE ? 's' : 'p',
		   0,//pgoff,
		   0, 0, 0);//MAJOR(dev), MINOR(dev), ino);
}

static void
show_map_vma(struct seq_file *m, struct vm_area_struct *vma, int is_pid)
{
	struct mm_struct *mm = vma->vm_mm;
	struct file *file = vma->vm_file;
	vm_flags_t flags = vma->vm_flags;
	unsigned long ino = 0;
	unsigned long long pgoff = 0;
	unsigned long start, end;
	dev_t dev = 0;
	const char *name = NULL;
	char path_buf[PATH_MAX] = {0};

	if (file) {
		struct inode *inode = file_inode(vma->vm_file);
		dev = inode->i_sb->s_dev;
		ino = inode->i_ino;
		pgoff = ((loff_t)vma->vm_pgoff) << PAGE_SHIFT;
	}

	start = vma->vm_start;
	end = vma->vm_end;
	if(file) {
		show_vma_header_prefix2(m, start, end, flags, pgoff, dev, ino);
	} else {
		show_vma_header_prefix(m, start, end, flags, pgoff, dev, ino);
	}	

	/*
	 * Print the dentry name for named mappings, and a
	 * special [heap] marker for the heap:
	 */
	if (file) {
        // 获取文件名称
		char *path = d_path(&file->f_path, path_buf, sizeof(path_buf));
		seq_pad(m, ' ');
		if(strlen(path) > 0) {            
			if(strstr(path, "xxxx.so")) {
                // 过滤xxxx.so
				goto done;
			}
		}				
		seq_file_path(m, file, "\n");		
		goto done;
	}

	if (vma->vm_ops && vma->vm_ops->name) {
		name = vma->vm_ops->name(vma);
		if (name)
			goto done;
	}

	name = arch_vma_name(vma);
	if (!name) {
		if (!mm) {
			name = "[vdso]";
			goto done;
		}

		if (vma->vm_start <= mm->brk &&
		    vma->vm_end >= mm->start_brk) {
			name = "[heap]";
			goto done;
		}

		if (is_stack(vma)) {
			name = "[stack]";
			goto done;
		}

		if (vma_get_anon_name(vma)) {
			seq_pad(m, ' ');
			seq_print_vma_name(m, vma);
		}
	}

done:
	if (name) {
		seq_pad(m, ' ');
		seq_puts(m, name);
	}
	seq_putc(m, '\n');
}
```

## 结果验证

通过上面的信息处理后, 所有的都没有对应的可执行权限了, 有了这种方法, 你就可以定制你自己的隐藏技术, maps文件便由你掌控

![20211014140205](https://cdn.jsdelivr.net/gh/yhnu/PicBed/20211014140205.png)
