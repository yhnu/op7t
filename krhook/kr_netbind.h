#ifndef KRHOOK_KR_NETBIND
#define KRHOOK_KR_NETBIND
#include <linux/in.h>
#include <linux/socket.h>

// linux/syscalls.h
typedef long (*TYPE_sk_bind)(int fd, struct sockaddr __user *umyaddr,
                             int addrlen);

static TYPE_sk_bind old_sk_bind;

long new_sk_bind(int fd, struct sockaddr __user *uaddr, int addrlen)
{
  unsigned long ret;
  struct sockaddr_in local_addr;
  memset(&local_addr, 0, sizeof(struct sockaddr_in));
  // check is sockaddr_in
  if ((ret = copy_from_user(&local_addr, uaddr, sizeof(struct sockaddr_in))))
  {
    printk("[err]krnet copy to usr error. err=%d\n", ret);
  }
  else
  {
    printk("[i]krnet size=%d addrlen=%d %d\n", sizeof(struct sockaddr_in), addrlen, ntohs(local_addr.sin_port));
    if (addrlen == sizeof(struct sockaddr_in))
    {
      const int pid = get_current()->pid;
      const int tgid = get_current()->tgid;
      printk("[i]krnet current->pid:[%d] tgid:[%d] port:[%d] \n", pid, tgid, ntohs(local_addr.sin_port));
    }
  }
  return old_sk_bind(fd, uaddr, addrlen);
}

#define KR_NETBIND_INIT()                                    \
  old_sk_bind = (TYPE_sk_bind)sys_call_table_ptr[__NR_bind]; \
  sys_call_table_ptr[__NR_bind] = (TYPE_sk_bind)new_sk_bind;

#define KR_NETBIND_CLEAN()                          \
  if (sys_call_table_ptr[__NR_bind] == new_sk_bind) \
  {                                                 \
    sys_call_table_ptr[__NR_bind] = old_sk_bind;    \
  }
#endif /* KRHOOK_KR_NETBIND */
