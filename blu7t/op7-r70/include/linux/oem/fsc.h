#ifndef _LINUX_FSC_H_
#define _LINUX_FSC_H_

#include <linux/fs.h>
#include <linux/sched.h>

#define FSC_PATH_MAX (256)
#define FSC_SCAN_BULK (512)
#define FSC_HASH_BUCKET (8192)
#define FSC_SUMMARY_MAX (128)

extern unsigned int fsc_enable;
extern int fsc_allow_list_cur;

extern void fsc_spin_lock(unsigned int hidx);
extern void fsc_spin_unlock(unsigned int hidx);
extern unsigned int fsc_get_hidx(const char* path, size_t len);
extern void fsc_insert_absence_path_locked(const char* path, size_t len, u32 hidx);
extern void fsc_delete_absence_path_locked(const char* path, size_t len, u32 hidx);
extern char *fsc_absolute_path(struct path* path, struct dentry* dentry, char *buf, size_t buflen);
extern void fsc_delete_absence_path_dentry(struct path*, struct dentry* dentry);
extern bool fsc_absence_check(const char* path, size_t len);
extern bool fsc_path_check(struct filename *name, size_t *len);
#endif
