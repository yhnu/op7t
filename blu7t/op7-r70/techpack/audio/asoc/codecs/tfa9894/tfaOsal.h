/*
 * Copyright 2014-2017 NXP Semiconductors
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
 * tfaOsal.c
 *
 *  Operating specifics
 */


#ifndef TFAOSAL_H_
#define TFAOSAL_H_

/* load hal plugin from dynamic lib */
void * tfaosal_plugload(char *libarg);
/*generic function to load library and symbol from the library*/
extern void * load_library(char *libname);
extern void * load_symbol(void *dlib_handle, char *dl_sym);

int tfaosal_filewrite(const char *fullname, unsigned char *buffer, int filelenght );

#define isSpaceCharacter(C) (C==' '||C=='\t')

#if defined (WIN32) || defined(_X64)
//suppress warnings for unsafe string routines.
#pragma warning(disable : 4996)
#pragma warning(disable : 4054)
char *basename(const char *fullpath);
#define bzero(ADDR,SZ)	memset(ADDR,0,SZ)
#endif

void tfaRun_Sleepus(int us);

/*
 * Read file
 */
int  tfaReadFile(char *fname, void **buffer);

#endif
