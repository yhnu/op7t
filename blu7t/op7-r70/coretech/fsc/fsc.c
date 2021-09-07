#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/stringhash.h>
#include <linux/fs.h>
#include <linux/mutex.h>

#include <linux/oem/fsc.h>

struct file_status_cache {
	char path[FSC_PATH_MAX];
	atomic_t refcnt;
	u64 last_scan_ts;
	u64 last_ref_ts;
	u64 first_ref_ts;
	u32 hidx;
	struct list_head node;
	int allow_idx; /* allow list index */
	atomic_t d_cnt; /* direct return cnt */
	u32 d_ret_ceil;
};

/* hashing algorithm */
struct fsc_hash {
	struct list_head head;
	atomic_t cnt;
	spinlock_t lock;
} fsc_htbl[FSC_HASH_BUCKET];

unsigned int fsc_enable = 0;
static bool fsc_ready = false;
static bool fsc_details = false;
module_param_named(details, fsc_details, bool, 0644);
static u32 fsc_d_ret = 128;
module_param_named(d_ret, fsc_d_ret, uint, 0644);

/* Set a ceil for fsc obj allocation */
unsigned int fsc_max_val = 100000;
module_param_named(fsc_max_val, fsc_max_val, uint, 0644);
static atomic_t fsc_cur_used;

/*
 * Allowed list
 * trailing character should not contain '/'
 * Interface for add/ del allow listi
 */
#define FSC_ALLOW_LIST_SIZE (32)

struct allow_obj {
	char *path;
	int idx;
	size_t len;
};

/* leave the last one always empty */
static struct allow_obj fsc_allow_list[FSC_ALLOW_LIST_SIZE + 1];
static int allow_idx_map[FSC_ALLOW_LIST_SIZE];
static atomic_t fsc_allow_list_cnt[FSC_ALLOW_LIST_SIZE];
int fsc_allow_list_cur;

static inline int get_empty_allow_idx(void) {
	int i = 0;
	for (; i < FSC_ALLOW_LIST_SIZE; ++i) {
		if (!allow_idx_map[i]) {
			allow_idx_map[i] = 1;
			return i;
		}
	}
	return -1;
}

/* To reclaim `out of date` fsc object */
struct fsc_reclaimer {
	struct list_head head;
	atomic_t cnt;
	struct task_struct *tsk;
	unsigned long period;
	unsigned int thres;
	char path[FSC_PATH_MAX];
} period, instcln;

void fsc_spin_lock(u32 hidx)
{
	spin_lock(&fsc_htbl[hidx].lock);
}

void fsc_spin_unlock(u32 hidx)
{
	spin_unlock(&fsc_htbl[hidx].lock);
}

/* dcache helper to get absolute path */
char *fsc_absolute_path(struct path* path, struct dentry* dentry, char *buf, size_t buflen)
{
	char localbuf[FSC_PATH_MAX] = {0};
	char *abspath = NULL;
	size_t rlen, dlen;

	if (!path || !dentry || dentry->d_name.name[0] == '\0')
		return NULL;

	abspath = d_absolute_path((const struct path*) path, localbuf, FSC_PATH_MAX);
	if (IS_ERR(abspath))
		return NULL;
	rlen = strlen(abspath);
	dlen = strlen(dentry->d_name.name);
	if (rlen + dlen + 2 > buflen)
		return NULL;
	memcpy(buf, abspath, rlen);
	buf[rlen] = '/';
	memcpy(buf + rlen + 1, dentry->d_name.name, dlen);
	buf[rlen + 1 + dlen] = '\0';

	pr_debug("%s get %s\n", __func__, buf);
	return buf;
}

/* Using kernel string hash algo to get hash value, and mod to fit bucket */
unsigned int fsc_get_hidx(const char* path, size_t len)
{
	u32 hidx = full_name_hash(NULL, path, len) % FSC_HASH_BUCKET;
	pr_debug("%s hashing str: %s to %u\n", __func__, path, hidx);
	return hidx;
}

/* Allock fsc object with initialed value */
static struct file_status_cache* fsc_alloc(const char* path, size_t len, u32 hidx, int allow_idx)
{
	struct file_status_cache* obj;
	unsigned int cur_used = atomic_read(&fsc_cur_used);

	if (cur_used >= fsc_max_val) {
		pr_debug("%s reach alloc max %u\n", __func__, cur_used);
		return NULL;
	}

	obj = kzalloc(sizeof(struct file_status_cache), GFP_NOWAIT);
	if (!obj) {
		pr_warn("%s create failed\n", __func__);
		return NULL;
	}

	/* init */
	strncpy(obj->path, path, len);
	obj->path[len] = '\0';
	atomic_set(&obj->refcnt, 1);
	obj->last_scan_ts = 0;
	obj->first_ref_ts = jiffies;
	obj->last_ref_ts = jiffies;
	obj->hidx = hidx;
	obj->d_ret_ceil = 1;
	obj->allow_idx = allow_idx;
	atomic_set(&obj->d_cnt, obj->d_ret_ceil);
	atomic_inc(&fsc_allow_list_cnt[obj->allow_idx]);
	/* For some apps, it will catch too much during installing stage, remove those records */
	if (likely(instcln.tsk) &&
		atomic_read(&fsc_allow_list_cnt[obj->allow_idx]) >= instcln.thres) {
		wake_up_process(instcln.tsk);
	}
	pr_debug("%s: %s %lu %p\n", __func__, obj->path, strlen(obj->path), obj);
	atomic_inc(&fsc_cur_used);
	return obj;
}

/* Free it */
static void fsc_free(struct file_status_cache* obj)
{
	pr_debug("%s: %p\n", __func__, obj);
	if (obj) {
		atomic_dec(&fsc_cur_used);
		atomic_dec(&fsc_allow_list_cnt[obj->allow_idx]);
		kfree(obj);
	}
}

static int fsc_allow_list_add_store(const char *buf, const struct kernel_param *kp)
{
	size_t len = strlen(buf);
	int i = 0, allow_idx = -1;
	char *path;

	if (!len)
		return 0;

	if (fsc_allow_list_cur >= FSC_ALLOW_LIST_SIZE) {
		pr_warn("allow list add failed due to reach list limitation\n");
		return -EINVAL;
	}

	path = kzalloc(len + 1, GFP_KERNEL);
	if (!path) {
		pr_err("memory allocation failed\n");
		return -EINVAL;
	}

	if (sscanf(buf, "%s", path) <= 0) {
		kfree(path);
		return -EINVAL;
	}

	len = strlen(path);
	for (i = 0; i < fsc_allow_list_cur && i < FSC_ALLOW_LIST_SIZE; ++i) {
		/* to avoid add duplicate list */
		if (strlen(fsc_allow_list[i].path) == len && !strncmp(fsc_allow_list[i].path, path, len)) {
			kfree(path);
			return 0;
		}
	}
	allow_idx = get_empty_allow_idx();
	if (allow_idx == -1) {
		pr_err("Can't get allow idx\n");
		kfree(path);
		return 0;
	}
	fsc_allow_list[fsc_allow_list_cur].path = path;
	fsc_allow_list[fsc_allow_list_cur].idx = allow_idx;
	fsc_allow_list[fsc_allow_list_cur].len = len;
	++fsc_allow_list_cur;

	return 0;
}

static struct kernel_param_ops fsc_allow_list_add_ops = {
	.set = fsc_allow_list_add_store,
};
module_param_cb(allow_list_add, &fsc_allow_list_add_ops, NULL, 0220);

static int fsc_allow_list_del_store(const char *buf, const struct kernel_param *kp)
{
	size_t len = strlen(buf);
	char *path;
	int i = 0, allow_idx;

	if (!len)
		return 0;

	path = kzalloc(len + 1, GFP_KERNEL);
	if (!path) {
		pr_err("memory allocation failed\n");
		return -EINVAL;
	}

	if (sscanf(buf, "%s", path) <= 0) {
		kfree(path);
		return -EINVAL;
	}

	len = strlen(path);
	for (i = 0; i < fsc_allow_list_cur && i < FSC_ALLOW_LIST_SIZE; ++i) {
		if (strlen(fsc_allow_list[i].path) == len && !strncmp(fsc_allow_list[i].path, path, len)) {
			kfree(path);
			path = fsc_allow_list[i].path;
			allow_idx = fsc_allow_list[i].idx;
			memset(instcln.path, '\0', FSC_PATH_MAX);
			strncpy(instcln.path, path, len);

			for (; i < fsc_allow_list_cur && i < FSC_ALLOW_LIST_SIZE; ++i) {
				fsc_allow_list[i].path = fsc_allow_list[i + 1].path;
				fsc_allow_list[i].idx = fsc_allow_list[i + 1].idx;
				fsc_allow_list[i].len = fsc_allow_list[i + 1].len;
			}
			--fsc_allow_list_cur;

			/* release obj */
			wake_up_process(instcln.tsk);
			allow_idx_map[allow_idx] = 0;
			break;
		}
	}
	kfree(path);
	return 0;
}

static struct kernel_param_ops fsc_allow_list_del_ops = {
	.set = fsc_allow_list_del_store,
};
module_param_cb(allow_list_del, &fsc_allow_list_del_ops, NULL, 0220);

static int fsc_dump_allow_list_show(struct seq_file *m, void *v)
{
	int i;

	seq_printf(m, "fsc allow list: Total %u\n", fsc_allow_list_cur);
	for (i = 0; i < fsc_allow_list_cur && i < FSC_ALLOW_LIST_SIZE; ++i)
		seq_printf(m, "%s, cnt: %u\n", fsc_allow_list[i].path, atomic_read(&fsc_allow_list_cnt[fsc_allow_list[i].idx]));
	return 0;
}

static int fsc_dump_allow_list_open(struct inode *inode, struct file *file)
{
	return single_open(file, fsc_dump_allow_list_show, NULL);
}

static const struct file_operations fsc_dump_allow_list_fops = {
	.open= fsc_dump_allow_list_open,
	.read= seq_read,
	.llseek= seq_lseek,
	.release= single_release,
};

static bool fsc_path_allow(const char* path, size_t len, int *allow_idx)
{
	int i = 0;
	size_t flen = 0;
	size_t offset = 0;

	/*
	 * enhance for this most frequency case: user 0
	 * /storage/emulated/0/Android/data/
	 */
#define FSC_ALLOW_COMMON_PREFIX "/storage/emulated/0/Android/data/"
#define FSC_ALLOW_COMMON_PREFIX_LEN (33)
	if (len < FSC_ALLOW_COMMON_PREFIX_LEN - 1) {
		/* at least len >= /storage/emulated//Android/data/ */
		return false;
	}

	if (strncmp(FSC_ALLOW_COMMON_PREFIX, path, FSC_ALLOW_COMMON_PREFIX_LEN))
		goto fsc_slow_check;

	/* fsc_fast_check only applied on user 0 */
	offset = FSC_ALLOW_COMMON_PREFIX_LEN;
	path += offset;
	len -= offset;
	for (i = 0; i < fsc_allow_list_cur && fsc_allow_list[i].path && i < FSC_ALLOW_LIST_SIZE; ++i) {
		char *check_path = fsc_allow_list[i].path + offset;
		flen = fsc_allow_list[i].len - offset;
		if (len >= flen && !strncmp(path, check_path, flen)) {
			*allow_idx = fsc_allow_list[i].idx;
			return true;
		}
	}
	return false;

fsc_slow_check:
	for (i = 0; i < fsc_allow_list_cur && fsc_allow_list[i].path && i < FSC_ALLOW_LIST_SIZE; ++i) {
		flen = fsc_allow_list[i].len;
		if (len >= flen && !strncmp(path, fsc_allow_list[i].path, flen)) {
			*allow_idx = fsc_allow_list[i].idx;
			return true;
		}
	}
	return false;
}

/*
 * To check if path already cached.
 * Calling with lock & unlock
 * Note:
 *   if cached, then
 *     inc refcnt
 *     update ref timestamp
 */
bool fsc_is_absence_path_exist_locked(const char* path, size_t len, u32 hidx, bool d_check)
{
	struct file_status_cache* fsc = NULL;
	int allow_idx = -1;

	if (list_empty(&fsc_htbl[hidx].head))
		return false;

	if (!fsc_path_allow(path, len, &allow_idx))
		return false;

	list_for_each_entry(fsc, &fsc_htbl[hidx].head, node) {
		if (strlen(fsc->path) == len && !strncmp(path, fsc->path, len)) {
			/* update ref status */
			fsc->last_ref_ts = jiffies;
			atomic_inc(&fsc->refcnt);
			/* direct return check */
			pr_debug("%s %s exits in bucket %u, d_check: %u, called from %pS\n",
				__func__, path, hidx, d_check, __builtin_return_address(0));
			if (d_check) {
				/* if d_cnt == 0, do real check; otherwise directly return */
				if (atomic_dec_and_test(&fsc->d_cnt)) {
					/* set next d_cnt by shift d_ret_ceil 1 */
					if (fsc->d_ret_ceil < fsc_d_ret)
						fsc->d_ret_ceil <<= 1;
					/* set direct return ceil */
					atomic_set(&fsc->d_cnt, fsc->d_ret_ceil);
					return false;
				}
			}
			return true;
		}
	}
	return false;
}

/* To check if path is good to apply fsc */
bool fsc_path_check(struct filename *name, size_t *len)
{
	const char *sc;

	if (name->name[0] != '/')
		return false;

	for (sc = name->name; *sc != '\0'; ++sc) {
		/* ignore // .. cases */
		if (sc != name->name && *sc == *(sc - 1) && (*sc == '/' || *sc == '.'))
			return false;
		/* ignore /./ case */
		else if (sc - name->name > 1 && *sc == '/' && *(sc - 1) == '.' && *(sc - 2) == '/')
			return false;
	}
	*len = sc - name->name;
	return name->name[*len - 1] != '/' && *len < FSC_PATH_MAX;
}

/*
 * To check if path already cahced.
 * Lock unlock inside
 */
bool fsc_absence_check(const char* path, size_t len)
{
	unsigned int hidx = 0;

	hidx = fsc_get_hidx(path, len);
	fsc_spin_lock(hidx);
	if (fsc_is_absence_path_exist_locked(path, len, hidx, true)) {
		fsc_spin_unlock(hidx);
		return true;
	}
	fsc_spin_unlock(hidx);
	return false;
}

/*
 * To insert absence path to hash table
 * Calling with lock & unlock
 * Check if file alread cached before calling
 */
void fsc_insert_absence_path_locked(const char* path, size_t len, u32 hidx)
{
	struct file_status_cache* fsc;
	int allow_idx = -1;

	if (!fsc_path_allow(path, len, &allow_idx))
		return;

	if (fsc_is_absence_path_exist_locked(path, len, hidx, false))
		return;

	fsc = fsc_alloc(path, len, hidx, allow_idx);
	if (!fsc)
		return;

	list_add(&fsc->node, &fsc_htbl[hidx].head);
	atomic_inc(&fsc_htbl[hidx].cnt);
	pr_debug("%s %s insert to bucket %u, called from %pS\n",
			__func__, path, hidx, __builtin_return_address(0));
}

/*
 * To delete absence path from hash table
 * Calling with lock & unlock
 */
void fsc_delete_absence_path_locked(const char* path, size_t len, u32 hidx)
{
	struct file_status_cache* fsc;
	int allow_idx = -1;

	if (!fsc_path_allow(path, len, &allow_idx))
		return;

	/* remove fsc obj from hashing list */
	list_for_each_entry(fsc, &fsc_htbl[hidx].head, node) {
		if (strlen(fsc->path) == len && !strncmp(path, fsc->path, len)) {
			atomic_dec(&fsc_htbl[hidx].cnt);
			list_del(&fsc->node);
			pr_debug("%s %s delete from bucket %u, called from %pS\n",
				__func__, path, hidx, __builtin_return_address(0));
			fsc_free(fsc);
			return;
		}
	}
}

/*
 * To delete absence path dentry from hash table
 * Lock and unlock inside function
 */
void fsc_delete_absence_path_dentry(struct path* path, struct dentry* dentry)
{
	char buf[FSC_PATH_MAX] = {0};
	char *abspath;

	abspath = fsc_absolute_path(path, dentry, buf, FSC_PATH_MAX);

	if (abspath) {
		size_t len = strlen(abspath);
		unsigned int hidx = fsc_get_hidx(abspath, len);
		fsc_spin_lock(hidx);
		fsc_delete_absence_path_locked(abspath, len, hidx);
		fsc_spin_unlock(hidx);
		pr_debug("%s %s delete from bucket %u, called from %pS\n",
			__func__, abspath, hidx, __builtin_return_address(0));
	}
}

/* summary */
struct fsc_summary {
	char path[FSC_PATH_MAX];
	char package[FSC_PATH_MAX];
	size_t len;
	size_t plen;
	u32 cnt;
};

static const char* prefix = "/storage/emulated/0/Android/data";
static struct fsc_summary* summary[FSC_SUMMARY_MAX];
static unsigned int summary_idx = 0;

/* Debug for dump hash tabl */
static unsigned int total_max;
static unsigned int len_max;
static int fsc_dump_htbl_proc_show(struct seq_file *m, void *v)
{
	int hidx;
	unsigned int total = 0;
	struct file_status_cache *fsc;
	size_t len, prefix_len = strlen(prefix);
	int i = 0;

	for (hidx = 0; hidx < FSC_HASH_BUCKET; ++hidx) {
		unsigned int cnt = 0;
		fsc_spin_lock(hidx);
		cnt = atomic_read(&fsc_htbl[hidx].cnt);
		total += cnt;
		if (!list_empty(&fsc_htbl[hidx].head)) {
			if (fsc_details)
				seq_printf(m, "hidx: %d, cnt: %d\n", hidx, cnt);
			/* cached list */
			list_for_each_entry(fsc, &fsc_htbl[hidx].head, node) {
				len = strlen(fsc->path);
				/* check if need summary */
				if (len > prefix_len && !strncmp(prefix, fsc->path, prefix_len) && summary_idx < FSC_SUMMARY_MAX) {
					char package[FSC_PATH_MAX] = {0};
					char *token = NULL, *end = NULL;
					memcpy(package, fsc->path, FSC_PATH_MAX);
					end = package + prefix_len;
					strsep(&end, "/");
					token = end;
					if (token) {
						size_t plen;
						bool need_alloc = true;
						strsep(&end, "/");
						if (end) {
							plen = strlen(token);

							/* check if need allocate summary object */
							for (i = 0; i < summary_idx; ++i) {
								if (plen == summary[i]->plen && !strncmp(token, summary[i]->package, plen)) {
									need_alloc = false;
									break;
								}
							}

							if (need_alloc) {
								summary[summary_idx] = kzalloc(sizeof(struct fsc_summary), GFP_NOWAIT);
								if (summary[summary_idx]) {
									memcpy(summary[summary_idx]->path, fsc->path, prefix_len + 1 + plen + 1);
									memcpy(summary[summary_idx]->package, token, plen);
									summary[summary_idx]->len = strlen(summary[summary_idx]->path);
									summary[summary_idx]->plen = plen;
									pr_debug("%s summary path: %s\n", __func__, summary[summary_idx]->path);
									pr_debug("%s summary package: %s\n", __func__, summary[summary_idx]->package);
									pr_debug("%s summary len: %lu, plen: %lu\n", __func__,
											summary[summary_idx]->len, summary[summary_idx]->plen);
									pr_debug("%s summary cnt: %u\n", __func__, summary[summary_idx]->cnt);
									++summary_idx;
								} else
									pr_warn("%s summary alloc failed\n", __func__);
							}
						}

						/* summary package information */
						for (i = 0; i < summary_idx; ++i) {
							if (len > summary[i]->len && !strncmp(fsc->path, summary[i]->path, summary[i]->len)) {
								++summary[i]->cnt;
								break;
							}
						}
					}
				}

				if (len > len_max)
					len_max = len;
				if (fsc_details)
					seq_printf(m, "  C:  %d %s %lu %llu %llu %u %u %d\n",
						atomic_read(&fsc->refcnt), fsc->path, strlen(fsc->path), fsc->first_ref_ts, fsc->last_ref_ts, fsc->d_ret_ceil, atomic_read(&fsc->d_cnt), fsc->allow_idx);
			}
		}
		fsc_spin_unlock(hidx);
	}
	if (total > total_max)
		total_max = total;
	seq_printf(m, "Cache: %u, Used: %u, history max: %u, len: %u, current: %lu\n",
		total, atomic_read(&fsc_cur_used), total_max, len_max, jiffies);

	if (summary_idx) {
		/* dump summary and reset summary */
		seq_printf(m, "Summary:\n");
		for (i = 0; i < summary_idx; ++i) {
			seq_printf(m, "\t[%d]: package: %s, absence file records: %u\n", i, summary[i]->package, summary[i]->cnt);
			if (summary[i])
				kfree(summary[i]);
		}
		summary_idx = 0;
	}
	return 0;
}

static int fsc_dump_htbl_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fsc_dump_htbl_proc_show, NULL);
}

static const struct file_operations fsc_dump_htbl_fops = {
	.open= fsc_dump_htbl_proc_open,
	.read= seq_read,
	.llseek= seq_lseek,
	.release= single_release,
};

static int fsc_enable_store(const char *buf, const struct kernel_param *kp)
{
	unsigned int val;
	int hidx;

	if (unlikely(!fsc_ready))
		return 0;

	if (sscanf(buf, "%u", &val) <= 0)
		return -EINVAL;

	fsc_enable = val;

	if (fsc_enable)
		return 0;

	/* if turn off, flush table */
	for (hidx = 0; hidx < FSC_HASH_BUCKET; ++hidx) {
		fsc_spin_lock(hidx);
		if (!list_empty(&fsc_htbl[hidx].head)) {
			struct file_status_cache *fsc, *tmp;
			list_for_each_entry_safe(fsc, tmp, &fsc_htbl[hidx].head, node) {
				if (fsc) {
					list_del(&fsc->node);
					fsc_free(fsc);
					atomic_dec(&fsc_htbl[hidx].cnt);
				}
			}
		}
		fsc_spin_unlock(hidx);
	}
	return 0;
}

static int fsc_enable_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u", fsc_enable);
}

static struct kernel_param_ops fsc_enable_ops = {
	.set = fsc_enable_store,
	.get = fsc_enable_show,
};
module_param_cb(enable, &fsc_enable_ops, NULL, 0644);

static int period_reclaim(void *arg)
{
	static unsigned int hidx = 0;
	while (!kthread_should_stop()) {
		int i = 0;

		if (!fsc_enable) {
			msleep(period.period);
			continue;
		}

		for (i = 0; i < FSC_SCAN_BULK; ++i) {
			hidx %= FSC_HASH_BUCKET;
			fsc_spin_lock(hidx);
			if (!list_empty(&fsc_htbl[hidx].head)) {
				struct file_status_cache *fsc, *tmp;
				list_for_each_entry_safe(fsc, tmp, &fsc_htbl[hidx].head, node) {
					if (fsc) {
						/* get rid of obj after 2 days (48 hours) passed */
						int delta = (jiffies - fsc->last_ref_ts)/(HZ * 172800L);
						if (delta) {
							/* time to go home */
							list_del(&fsc->node);
							atomic_dec(&fsc_htbl[hidx].cnt);
							list_add(&fsc->node, &period.head);
							pr_debug("%s decay: %s out\n", __func__, fsc->path);
						}
					}
				}
			}
			fsc_spin_unlock(hidx);
			++hidx;
		}

		/* free them */
		if (!list_empty(&period.head)) {
			struct file_status_cache *fsc, *tmp;
			list_for_each_entry_safe(fsc, tmp, &period.head, node) {
				list_del(&fsc->node);
				fsc_free(fsc);
			}
		}
		msleep(period.period);
	}
	return 0;
}

/* scan for out of limit fsc object */
static int instcln_reclaim(void* args)
{
	unsigned int i = 0;
	char *uninst_pkg = (char *)args;
	int uninst_len;
	bool uninst_ing = false;
	bool remove = false;
	int target;

	while (!kthread_should_stop()) {
		/* always reset target before really reclaim */
		target = -1;

		if (!fsc_enable) {
			usleep_range(instcln.period, instcln.period + 1000000);
			continue;
		}

		/* step 1. find out which package is going to uninstall */
		uninst_len = strlen(uninst_pkg);
		if (unlikely(uninst_len)) {
			uninst_ing = true;
			goto redo;
		}

		/* step 2. find out which package over limit */
		for (i = 0; target == -1 && i < fsc_allow_list_cur; ++i) {
			if (atomic_read(&fsc_allow_list_cnt[fsc_allow_list[i].idx]) >= instcln.thres) {
				target = i;
				break;
			}
		}

		if (target == -1)
			goto done;

redo:
		/* step 3. reclaim objs */
		for (i = 0; i < FSC_HASH_BUCKET; ++i) {
			fsc_spin_lock(i);
			/* drop target fsc object */
			if (!list_empty(&fsc_htbl[i].head)) {
				struct file_status_cache *fsc, *tmp;
				list_for_each_entry_safe(fsc, tmp, &fsc_htbl[i].head, node) {
					if (likely(fsc)) {
						remove = false;
						if (uninst_ing && !strncmp(fsc->path, uninst_pkg, uninst_len))
							remove = true;
						else if (!uninst_ing && fsc->allow_idx == target)
							remove = true;
						if (remove) {
							list_del(&fsc->node);
							atomic_dec(&fsc_htbl[i].cnt);
							list_add(&fsc->node, &instcln.head);
						}
					}
				}
			}
			fsc_spin_unlock(i);
		}

		if (!list_empty(&instcln.head)) {
			struct file_status_cache *fsc, *tmp;
			list_for_each_entry_safe(fsc, tmp, &instcln.head, node) {
				list_del(&fsc->node);
				fsc_free(fsc);
			}
		}

		/* step 4. check if any uninstall event happened during reclaim period */
		if (uninst_ing)
			uninst_pkg[0] = '\0';
		uninst_len = strlen(uninst_pkg);
		if (unlikely(uninst_len)) {
			uninst_ing = true;
			goto redo;
		}
done:
		uninst_ing = false;
		usleep_range(instcln.period, instcln.period + 1000000);
	}
	return 0;
}

static int __init fsc_init(void)
{
	int i = 0;
	pr_info("%s\n", __func__);

	atomic_set(&fsc_cur_used, 0);
	for (i = 0; i < FSC_HASH_BUCKET; ++i) {
		INIT_LIST_HEAD(&fsc_htbl[i].head);
		atomic_set(&fsc_htbl[i].cnt, 0);
		spin_lock_init(&fsc_htbl[i].lock);
	}

	for (i = 0; i < FSC_ALLOW_LIST_SIZE; ++i)
		atomic_set(&fsc_allow_list_cnt[i], 0);

	/* init for period & instcln reclaimer */
	INIT_LIST_HEAD(&period.head);
	period.period = 1 * 60 * 60 * 1000; // 1 hour, ms
	period.tsk = kthread_run(period_reclaim, NULL, "fsc_period_reclaimer");
	if (!period.tsk) {
		pr_err("%s: init period reclaimer failed\n", __func__);
		return 0;
	}

	INIT_LIST_HEAD(&instcln.head);
	instcln.period = 1L * 60L * 60L * 1000L * 1000L; // 1 hour, us
	instcln.thres = 10000; // per package should not cache more than this thres.
	instcln.path[0] = '\0';;
	instcln.tsk = kthread_run(instcln_reclaim, instcln.path, "fsc_instcln_reclaimer");
	if (!instcln.tsk) {
		pr_err("%s: init instcln reclaimer failed\n", __func__);
		kthread_stop(period.tsk);
		return 0;
	}

	/* info */
	proc_create("fsc_dump", S_IFREG | 0400, NULL, &fsc_dump_htbl_fops);
	proc_create("fsc_allow_list", S_IFREG | 0400, NULL, &fsc_dump_allow_list_fops);

	pr_info("%s done\n", __func__);

	fsc_ready = true;
	return 0;
}
pure_initcall(fsc_init);
