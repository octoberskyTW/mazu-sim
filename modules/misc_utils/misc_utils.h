
#ifndef __MISC_UTILS_H__
#define __MISC_UTILS_H__

#define UNUSED(expr)   \
    do {               \
        (void) (expr); \
    } while (0)

#ifdef __cplusplus
extern "C" {
#endif
void timestamp(char *currentTime);
void hex_dump(char *str, uint8_t *pSrcBufVA, uint32_t SrcBufLen);
#ifdef __cplusplus
}
#endif

#endif  //  __MISC_UTILS_H__
