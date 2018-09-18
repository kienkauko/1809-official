#include "rgb2gray.hpp"

void rgbtogray (uint8_t red[307200], uint8_t green[307200], uint8_t blue[307200], uint8_t *gray_p){
    // MINE
	uint8_t *gray = gray_p;
	uint8_t *r=red;
	uint8_t *g=green;
	uint8_t *b=blue;

	uint16x8_t tem;
	uint8x8_t result;

	uint8x8_t red_v, green_v, blue_v;
	uint8x8_t rfac = vdup_n_u8(77); 			//0.2989*256 de sau chia 256 <=> shift 8
	uint8x8_t gfac = vdup_n_u8(150); 			//0.5870*256 de sau chia 256 <=> shift 8
	uint8x8_t bfac = vdup_n_u8(29); 			//0.1140*256 de sau chia 256 <=> shift 8
		for(int j=0;j<38400;j++, gray+=8, r+=8, g+=8, b+=8){

			red_v = vld1_u8(r);
			green_v = vld1_u8(g);
			blue_v = vld1_u8(b);

			// Multiply all eight red pixels by the corresponding weights.
			tem = vmull_u8 (red_v, rfac);
			// Combined multiply and additionf
			tem = vmlal_u8 (tem, green_v, gfac);
			tem = vmlal_u8 (tem, blue_v, bfac);
			// Shift right by 8, "narrow" to 8-bits (recall temp is 16-bit).
			result = vshrn_n_u16 (tem, 8);
			// Store converted pixels in the output grayscale image.
			vst1_u8(gray, result);
		}

}
