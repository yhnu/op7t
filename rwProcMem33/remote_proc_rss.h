#ifndef RWPROCMEM33_REMOTE_PROC_RSS
#define RWPROCMEM33_REMOTE_PROC_RSS
#include <linux/pid.h>
size_t read_proc_rss_size(struct pid *proc_pid_struct, ssize_t relative_offset);
#endif /* RWPROCMEM33_REMOTE_PROC_RSS */
