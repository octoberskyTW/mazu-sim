#include <stdio.h>

void say_hello(void) {
    printf("[dungru:%d:%s] \n", __LINE__, __FUNCTION__);
}

void say_hello_1(void) {
    printf("[dungru:%d:%s] \n", __LINE__, __FUNCTION__);
}