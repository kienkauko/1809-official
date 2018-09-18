#include <iostream>
#include <arm_neon.h>

using namespace std;

#define alpha 0.01
#define k 3

float32x4_t invsqrtv(float32x4_t x);
float32x4_t sqrtv(float32x4_t x);
void background_initiate(uint8_t *frame_ini, float32_t *muy, float32_t *var);
uint32_t findDensity(uint8_t *frame, float32_t *muy, float32_t *var);
uint32_t findDensity_test(uint8_t *frame, float32_t *muy, float32_t *var, uint8_t *fg);

