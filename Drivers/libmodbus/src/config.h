/* config.h - Configuration for libmodbus on embedded systems */

#ifndef CONFIG_H
#define CONFIG_H

/* Define if you have strerror_r */
#define HAVE_STRERROR_R 0

/* Define if you have strlcpy */
#define HAVE_STRLCPY 0

/* Define if you have clock_gettime */
#define HAVE_CLOCK_GETTIME 0

/* Define if you have getaddrinfo */
#define HAVE_GETADDRINFO 0

/* Define if you have inet_ntop */
#define HAVE_INET_NTOP 0

/* Define if you have inet_pton */
#define HAVE_INET_PTON 0

/* Define if you have poll */
#define HAVE_POLL 0

/* For embedded systems, we might not have unistd.h */
#ifndef _MSC_VER
/* Assume we have unistd.h for now, but this might need adjustment */
#define HAVE_UNISTD_H 1
#endif

/* Disable TCP support for embedded */
#define HAVE_INET_H 0

/* Disable termios for embedded serial */
#define HAVE_TERMIOS_H 0

#endif /* CONFIG_H */