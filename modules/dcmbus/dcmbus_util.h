#ifndef __DCMBUS_UTILITY_H_
#define __DCMBUS_UTILITY_H_
#include "linux_common.h"
#define DCMBUS_DEBUG_ENABLE 0
/* Must can devide by 8*/
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))
#define BILLION 1000000000L
#define FTRACE_TIME_STAMP(id) \
    do {                      \
        syscall(id);          \
    } while (0)

#define BIT(n) ((0x1U) << (n))
#define BITS(m, n) (~(BIT(m) - 1) & ((BIT(n) - 1) | BIT(n)))

#define OUT_RANGE(i, min, max) ((i < min) || (i > max) ? 1 : 0)

#define SWAP32(x)                                                   \
    ((uint32_t)((((uint32_t)(x) & (uint32_t) 0x000000ffUL) << 24) | \
                (((uint32_t)(x) & (uint32_t) 0x0000ff00UL) << 8) |  \
                (((uint32_t)(x) & (uint32_t) 0x00ff0000UL) >> 8) |  \
                (((uint32_t)(x) & (uint32_t) 0xff000000UL) >> 24)))

#define cpu2le32(x) SWAP32((x))

#define errExit(msg) \
    do {             \
        perror(msg); \
        abort();     \
    } while (0)

#define debug_print(...)                  \
    do {                                  \
        if (DCMBUS_DEBUG_ENABLE)          \
            fprintf(stderr, __VA_ARGS__); \
    } while (0)
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define PP_HTONS(x)                                 \
    ((uint16_t)((((x) & (uint16_t) 0x00ffU) << 8) | \
                (((x) & (uint16_t) 0xff00U) >> 8)))
#define PP_NTOHS(x) PP_HTONS(x)
#define PP_HTONL(x)                            \
    ((((x) & (uint32_t) 0x000000ffUL) << 24) | \
     (((x) & (uint32_t) 0x0000ff00UL) << 8) |  \
     (((x) & (uint32_t) 0x00ff0000UL) >> 8) |  \
     (((x) & (uint32_t) 0xff000000UL) >> 24))
#define PP_NTOHL(x) PP_HTONL(x)
#else /* __ORDER_BIG_ENDIAN__ */
#define PP_HTONS(x) (x)
#define PP_NTOHS(x) (x)
#define PP_HTONL(x) (x)
#define PP_NTOHL(x) (x)
#endif
/**
 * @brief   TiSPACE EGSE System Profiling Server (ESPS)
 */

#define TVC_SIZE 6
#define RCS_SIZE 4
#define II_VALUE_CONTROL_SIZE 3
#define ORDANCE_SIZE 3

struct esps2egse_header_t {
    uint32_t payload_len;
    uint32_t crc;
} __attribute__((packed));
#define ESPS2EGSE_HEADER_SIZE (sizeof(struct esps2egse_header_t))

/*
 *
 * EGSE->DM 20pps, RX cmd format from devices. (egse data verify server)
 *
 */
struct esps2egse_data_t {
#if CONFIG_ESPS_HEADER_ENABLE
    uint8_t III_TVC_1[TVC_SIZE];
    uint8_t III_TVC_2[TVC_SIZE];
    uint8_t III_valve_control_1[II_VALUE_CONTROL_SIZE];
    uint8_t III_valve_control_2[II_VALUE_CONTROL_SIZE];
    uint8_t RCS[RCS_SIZE];
    uint8_t ordance_faring[ORDANCE_SIZE];
    uint8_t ordance_separation[ORDANCE_SIZE];
    uint8_t II_TVC_1[TVC_SIZE];
    uint8_t II_TVC_2[TVC_SIZE];
    uint8_t II_valve_control_1[II_VALUE_CONTROL_SIZE];
    uint8_t II_valve_control_2[II_VALUE_CONTROL_SIZE];
    uint8_t reserve[2];
#else
    uint8_t single_cmd[8];
#endif
} __attribute__((packed));

#ifdef __cplusplus
extern "C" {
#endif
uint32_t crc32(uint32_t crc, const uint8_t *buf);
uint32_t crc32_create(const uint8_t *buf, const uint32_t len);
void hex_dump(char *str, uint8_t *pSrcBufVA, uint32_t SrcBufLen);
void debug_hex_dump(char *str, uint8_t *pSrcBufVA, uint32_t SrcBufLen);
uint32_t invert_crc32(uint32_t crc);
uint32_t crc_checker(uint32_t rx_crc, const uint8_t *buf, uint32_t size);
double get_curr_time(void);
int copy_buffer_htons(uint8_t *dest, uint16_t *src);
int copy_buffer_ntohs(uint16_t *dest, uint8_t *src);
int copy_buffer_htonl(uint8_t *dest, uint32_t *src);
int copy_buffer_ntohl(uint32_t *dest, uint8_t *src);
int16_t TRUNCAT_16BIT(double x);
int32_t ROUND_32BIT(double x);
#ifdef __cplusplus
}
#endif

#endif  //  __DCMBUS_UTILITY_H_
