#ifndef __LINUX_COMMON_H__
#define __LINUX_COMMON_H__

#include <errno.h> /* Error number definitions */
#include <fcntl.h> /* File control definitions */
#include <stdint.h>
#include <stdio.h> /* Standard input/output definitions */
#include <stdlib.h>
#include <string.h>  /* String function definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>  /* UNIX standard function definitions */

#include <arpa/inet.h>
#include <endian.h>
#include <limits.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#endif  //  __LINUX_COMMON_H__