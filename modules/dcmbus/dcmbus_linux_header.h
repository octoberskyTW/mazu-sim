#ifndef __DCMBUS_LINUX_HEADER_H__
#define __DCMBUS_LINUX_HEADER_H__

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>
#include <stdint.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <time.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <sys/select.h>
#include <endian.h>
#include <limits.h>
#endif  //  __DCMBUS_LINUX_HEADER_H__