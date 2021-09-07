/*
 * Copyright 2012-2017 NXP Semiconductors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * tfaContUtil.h
 *
 *  interface to the Tfa98xx API
 *
 *  Created on: Apr 6, 2012
 *      Author: wim
 */

#ifndef TFACONTUTIL_H_
#define TFACONTUTIL_H_

#ifdef __ANDROID__
#include <utils/Log.h>
#else
#define LOGV if (0/*tfa98xx_verbose*/) printf //TODO improve logging
#endif

#include "tfa_dsp_fw.h"
#include "tfa98xx_parameters.h"

#if defined(WIN32) || defined(_X64)
#include <windows.h>
#else
#include <sys/time.h>
#include <signal.h>
#endif

#ifndef timer_INCLUDED
#define timer_INCLUDED
#endif

#if defined(__cplusplus)
extern "C" {
#endif

#define NXPTFA_MAXLINE		(256)       /* maximum string length */
#define NXPTFA_MAXBUFFER	(50*1024)   /* maximum buffer size to hold the container */

/*
 * buffer types for setting parameters
 */
typedef enum nxpTfa98xxParamsType {
    tfa_patch_params,
    tfa_speaker_params,
    tfa_preset_params,
    tfa_config_params,
    tfa_equalizer_params,
    tfa_drc_params,
    tfa_vstep_params,
    tfa_cnt_params,
    tfa_msg_params,
    tfa_no_params,
	tfa_info_params,
	tfa_algo_params
} nxpTfa98xxParamsType_t;

enum Tfa98xx_Error tfaVersions(int dev_idx, char *strings, int maxlength);
int tfa98xxSaveFile(int dev_idx, char *filename, nxpTfa98xxParamsType_t params);
enum Tfa98xx_Error tfaGetDspFWAPIVersion(int dev_idx);
int tfaGetRegMapVersion(int dev_idx, char *buffer);
int compare_strings(char *buffer, char *name, int buffersize);
enum Tfa98xx_Error tfaVersion(char *buffer);

/*
 * save dedicated device files. Depends on the file extension
 */
int tfa98xxSaveFileWrapper(int dev_idx, char *filename);

enum Tfa98xx_Error tfa98xx_verify_speaker_range(int idx, float imp[2], int spkr_count);

/* hex dump of cnt */
void tfa_cnt_hexdump(void);
void tfa_cnt_dump(void);

/* Timer functions (currently only used by livedata) */
int start_timer(int, void (*)(void));
void stop_timer(void);

void extern_msleep_interruptible(int msec);

enum Tfa98xx_Error tfa_open(int maxdev);
void tfa_close(int maxdev);

int is_ext_dsp(int dev_idx);
enum Tfa98xx_Error tfa_send_calibration_values();
enum Tfa98xx_Error tfa_start(int next_profile, int vstep);
enum Tfa98xx_Error tfa_stop();

enum Tfa98xx_Error tfa_system_open(unsigned char *slave_address, int count);
#if defined(__cplusplus)
}  /* extern "C" */
#endif
#endif /* TFACONTUTIL_H_ */
