/*
 * oem_force_dump.h
 *
 * header file supporting debug functions for Oneplus device.
 *
 * hefaxi@filesystems, 2015/07/03.
 */

#ifndef OEM_FORCE_DUMP_H
#define OEM_FORCE_DUMP_H

extern void oem_check_force_dump_key(unsigned int code, int value);
extern int oem_get_download_mode(void);
void send_msg(char *message);
int  msm_serial_oem_init(void);
#endif
