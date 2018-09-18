#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <jpeglib.h>
#include <jerror.h>
#include <arm_neon.h>

void jpeg_decode(unsigned char *buffer, uint8_t red[307200], uint8_t green[307200], uint8_t blue[307200]);

