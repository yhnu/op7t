/*
 *Copyright (c)  2018  OnePlus Mobile Comm Corp., Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _FG_UID_H
#define _FG_UID_H

#include <linux/spinlock.h>

#define MAX_ARRAY_LENGTH 256
#define FS_FG_INFO_PATH "fg_info"
#define FS_FG_UIDS "fg_uids"

struct fg_info {
	int fg_num;
	int fg_uids;
};

#endif /*_FG_UID_H*/
