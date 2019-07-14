#ifndef __CONFIG_UTIL_H__ 
#define __CONFIG_UTIL_H__

#include <stdint.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>

typedef enum __CONFIG_STATUS { 
    CONFIG_ERROR = -1,
    CONFIG_SUCCESS = 0
} CONFIG_STATUS_E; 

typedef enum __CONFIG_SPECIFIER { 
    CONFIG_STRING = 0,
    CONFIG_ENUM   = 1,
    CONFIG_NUMBER = 2
} CONFIG_SPECIFIER_E; 

#ifdef __cplusplus
extern "C" {
#endif
int read_config(void *table, int *numEntries, const char* config_path, char *specifier);
#ifdef __cplusplus
}
#endif


#endif  //  __CONFIG_UTIL_H__