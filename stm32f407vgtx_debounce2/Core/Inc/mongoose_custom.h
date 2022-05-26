#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdint-gcc.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <lwip/errno.h>

#define LWIP_IPV4 1

#include <lwip/sockets.h>

#define MG_DIRSEP '/'
#define MG_INT64_FMT "%lld"
