#ifndef KRHOOK_KR_NETSEND
#define KRHOOK_KR_NETSEND
#include <linux/in.h>
#include <linux/socket.h>

// asmlinkage long sys_sendto(int, void __user *, size_t, unsigned, struct
// sockaddr __user *, int);

// linux/syscalls.h
typedef long (*TYPE_sk_sendto)(int socket, void __user* message, size_t length,
                               unsigned flags,
                               struct sockaddr __user* dest_addr, int dest_len);

static TYPE_sk_sendto old_sk_sendto;

long new_sk_sendto(int socket, void __user* message, size_t length,
                   unsigned flags, struct sockaddr __user* dest_addr,
                   int dest_len) {
  if (dest_len > 0) {
    unsigned long ret;
    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(struct sockaddr_in));

    // check is sockaddr_in
    if ((ret = copy_from_user(&local_addr, dest_addr,
                              sizeof(struct sockaddr_in)))) {
      printk("[err]krnet sendto copy to usr error. err=%d dest_len=%d\n", ret,
             dest_len);
    } else {
      printk("[i]krnet sendto size=%d addrlen=%d %d\n",
             sizeof(struct sockaddr_in), dest_len, ntohs(local_addr.sin_port));
    }
  }

  return old_sk_sendto(socket, message, length, flags, dest_addr, dest_len);
}

#define KR_NETSEND_INIT()                                          \
  old_sk_sendto = (TYPE_sk_sendto)sys_call_table_ptr[__NR_sendto]; \
  sys_call_table_ptr[__NR_sendto] = (TYPE_sk_sendto)new_sk_sendto;

#define KR_NETSEND_CLEAN()                                \
  if (sys_call_table_ptr[__NR_sendto] == new_sk_sendto) { \
    sys_call_table_ptr[__NR_sendto] = old_sk_sendto;      \
  }

#endif /* KRHOOK_KR_NETSEND */
