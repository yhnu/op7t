/****************************************************************************************
** Copyright (C), 2013-2018, ONEPLUS Mobile Comm Corp., Ltd
** File: oneplus_motor_notifier.H
**
** Description:
**      Definitions for motor  notifier.
**
****************************************************************************************/

#ifndef _ONEPLUS_MOTOR_NOTIFIER
#define _ONEPLUS_MOTOR_NOTIFIER

#include <linux/notifier.h>

enum motor_event {
	MOTOR_UP_EVENT = 0,
	MOTOR_DOWN_EVENT,
	MOTOR_BLOCK_EVENT,
};

extern int register_motor_notifier(struct notifier_block *nb);
extern int unregister_motor_notifier(struct notifier_block *nb);
extern int motor_notifier_call_chain(unsigned long val);

#endif //_ONEPLUS_MOTOR_NOTIFIER