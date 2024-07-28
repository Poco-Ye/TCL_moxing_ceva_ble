/*
 * Copyright (c) 2009-2018 Arm Limited. All rights reserved.
 * Copyright (c) 2019 Nuclei Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/******************************************************************************
 * @file     common.c
 * @brief    NMSIS Nuclei Device Peripheral Access Layer Source File for
 *           Nuclei N Device
 * @version  V1.10
 * @date     30. July 2021
 ******************************************************************************/
#include <nmsis_core.h>
#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>

#undef errno
extern int errno;

__WEAK int _close(int fd)
{
    errno = EBADF;
    return -1;
}
__WEAK void _exit(int fd)
{
    while(1) {
        __WFI();
    }
}
__WEAK int _fork(void)
{
    errno = EAGAIN;
    return -1;
}
__WEAK int _fstat(int file, struct stat *st)
{
    if ((STDOUT_FILENO == file) || (STDERR_FILENO == file)) {
        st->st_mode = S_IFCHR;
        return 0;
    } else {
        errno = EBADF;
        return -1;
    }
}
__WEAK int _lseek(int file, int offset, int whence)
{
    return 0;
}

__WEAK int _open(const char *name, int flags, int mode)
{
    errno = ENOSYS;
    return -1;
}

__WEAK ssize_t _read(int fd, void* ptr, size_t len)
{

    return 1;
}


__WEAK int _wait(int *status)
{
    errno = ECHILD;
    return -1;
}

__WEAK ssize_t _write(int fd, const void* ptr, size_t len)
{

}

__WEAK int _isatty(int fd)
{
    return 1;
}

//added by jiangde
void wait_nop(unsigned int n)
{
	unsigned int i;

    for (i = 0; i < n; i++) {
        asm("nop");
    }

}


