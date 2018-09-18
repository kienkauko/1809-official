#include <time.h>
#include <iostream>
#include "arm_neon.hpp"

using namespace std;

void rgbtogray (uint8_t red[307200], uint8_t green[307200], uint8_t blue[307200], uint8_t *gray_p);
