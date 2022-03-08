#ifndef KRHOOK_KR_KILL
#define KRHOOK_KR_KILL

// #include <sys/types.h>
// linux/syscalls.h
typedef long (*TYPE_kill)(pid_t pid, int sig);

static TYPE_kill old_kill;

// https://github.com/yhnu/op7t/blob/dev/blu7t/op7-r70/security/apparmor/include/sig_names.h

long new_kill(pid_t pid, int sig) {      
  PRINT_STACK("[info]kr_kill ", "sig=%d uid=%d", sig, current_uid());
  return old_kill(pid, sig);
}

#define KR_KILL_INIT()                                 \
  old_kill = (TYPE_kill)sys_call_table_ptr[__NR_kill]; \
  sys_call_table_ptr[__NR_kill] = (TYPE_kill)new_kill;

#define KR_KILL_CLEAN()                            \
  if (sys_call_table_ptr[__NR_kill] == new_kill) { \
    sys_call_table_ptr[__NR_kill] = old_kill;      \
  }

#endif /* KRHOOK_KR_KILL */
