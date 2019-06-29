#include <stdio.h>

void say_hello(void) {
    printf("[dungru:%d:%s] \n", __LINE__, __FUNCTION__);
}