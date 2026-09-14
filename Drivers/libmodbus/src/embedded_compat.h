/* embedded_compat.h - Compatibility functions for embedded systems */

#ifndef EMBEDDED_COMPAT_H
#define EMBEDDED_COMPAT_H

#include <stdint.h>
#include <time.h>

/* Network byte order functions (big-endian) */
static inline uint16_t htons(uint16_t hostshort) {
    return ((hostshort & 0xFF) << 8) | ((hostshort >> 8) & 0xFF);
}

static inline uint16_t ntohs(uint16_t netshort) {
    return htons(netshort);
}

static inline uint32_t htonl(uint32_t hostlong) {
    return ((hostlong & 0xFF) << 24) | ((hostlong & 0xFF00) << 8) |
           ((hostlong & 0xFF0000) >> 8) | ((hostlong & 0xFF000000) >> 24);
}

static inline uint32_t ntohl(uint32_t netlong) {
    return htonl(netlong);
}

/* Sleep function - for embedded, this would need to be implemented */
static inline int nanosleep(const struct timespec *req, struct timespec *rem) {
    /* Stub implementation - in real embedded code, use HAL_Delay or similar */
    (void)req;
    (void)rem;
    return 0;
}

#endif /* EMBEDDED_COMPAT_H */