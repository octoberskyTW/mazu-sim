#ifndef __JIT_UTILS_H__
#define __JIT_UTILS_H__

#define IS_FLIGHT_EVENT_ARRIVED(event, bitmap, code) ((bitmap) & (0x1U << (event)) && (code) == (event))

#define PRINT_FLIGHT_EVENT_MESSAGE(side, sim_time, text, code)                            \
    do {                                                                                  \
        fprintf(stderr, "[%s:%s:%f] %s: %d\n", side, __FUNCTION__, sim_time, text, code); \
    } while (0)
#endif  // __JIT_UTILS_H__
