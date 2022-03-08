#ifndef KRHOOK_KR_PTRACE
#define KRHOOK_KR_PTRACE

typedef int (*TYPE_ptrace)(int request, int pid, int address, int data);

static TYPE_ptrace old_ptrace;

// ref
// https://github.com/yhnu/op7t/blob/dev/blu7t/op7-r70/include/uapi/linux/ptrace.h
// Stupid enum so we aren't staring
const char *stringFromPtrace(int request) {
  static const char *strings[25] = {
      "PTRACE_TRACEME", "PTRACE_PEEKTEXT", "PTRACE_PEEKDATA", "PTRACE_PEEKUSR",
      "PTRACE_POKETEXT", "PTRACE_POKEDATA", "PTRACE_POKEUSR", "PTRACE_CONT",
      "PTRACE_KILL", "PTRACE_SINGLESTEP",
      // unknown
      "UNK", "UNK", "UNK", "UNK", "UNK", "UNK", "PTRACE_ATTACH",
      "PTRACE_DETACH",
      // unknown
      "UNK", "UNK", "UNK", "UNK", "UNK", "UNK", "PTRACE_SYSCALL"};
  if (request < 25) return strings[request];
  return "UNK";
}

// Is this required? Only causes warning and doesn't seem to matter
// extern struct task_struct *current(void);
int ignore_ptrace_requests = 0;

// Hooked ptrace function
int new_ptrace(int request, int pid, int address, int data) {
  /*
  int ret = 0;

  // For various reasons this can be useful, just send a ptrace function with
  this value to
  // ignore the rest of the ptraces
  if (data == 0xFEEDD1FF)
  {
      ignore_ptrace_requests = 1;
  }

  if (current->ptrace & PT_PTRACED || ignore_ptrace_requests)
  {
      // If someone is being ptraced and asks to be ptraced,
      // just tell them they are instead of returning < 0
      printk("Force feeding 0 back to pid...\n");
      ret = 0;
  }
  else
  {
      // pass to real ptrace
      ret = old_ptrace(request, pid, address, data);
  }

  return ret;
  */
  printk(KERN_INFO "Ptrace was called; request[%d] pid[%d] addr[%x] data[%x]\n",
         request, pid, address, data);
  return old_ptrace(request, pid, address, data);
}

#define KRPTRACE_INIT()                                      \
  old_ptrace = (TYPE_ptrace)sys_call_table_ptr[__NR_ptrace]; \
  sys_call_table_ptr[__NR_ptrace] = (TYPE_ptrace)new_ptrace;

#define KRPTRACE_CLEAN()                               \
  if (sys_call_table_ptr[__NR_ptrace] == new_ptrace) { \
    sys_call_table_ptr[__NR_ptrace] = old_ptrace;      \
  }

#endif /* KRHOOK_KR_PTRACE */
