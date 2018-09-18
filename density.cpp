#include "density.hpp"

uint8x8_t ini;
uint16x8_t ini1;
uint16x4_t ini1_high, ini1_low;
uint32x4_t ini1_high_long, ini1_low_long;
float32x4_t ini1_high_f, ini1_low_f;
//create initial values from first 5 frames
//Input: 5 frames - 1d array -> frame_ini[5*640*480]
//
// approximative quadword float inverse square root
// 1 quadword = 4 lan do lon cua 1 word, 64 bits 
float32x4_t invsqrtv(float32x4_t x) {
    float32x4_t sqrt_reciprocal = vrsqrteq_f32(x);
    float32x4_t a = vmulq_f32(x, sqrt_reciprocal);
    float32x4_t b = vrsqrtsq_f32(a, sqrt_reciprocal);
    float32x4_t result = vmulq_f32(b, sqrt_reciprocal);
    return result;
    //return vrsqrtsq_f32(x * sqrt_reciprocal, sqrt_reciprocal) * sqrt_reciprocal;
}

// approximative quadword float square root
float32x4_t sqrtv(float32x4_t x) {
    float32x4_t inv = invsqrtv(x);
    float32x4_t result = vmulq_f32(x, inv);
    return result;
}

void background_initiate(uint8_t *frame_ini, float32_t *muy, float32_t *var){
	uint8_t *frame_ini_p = frame_ini;
	float32x4_t dif_high, dif_low;
	float32_t *muy_p = muy;
	float32x4_t muy_high_v, muy_low_v;
	float32_t *var_p = var;
	float32x4_t var_high_v, var_low_v;
	//calculate muy
	for(int m=0; m<5; m++){
		for(int i=0;i<480;i++){
			for(int j=0;j<80;j++){
				ini = vld1_u8(frame_ini_p);
				frame_ini_p+=8;
				//convert to float
				ini1 = vmovl_u8(ini);
				ini1_high = vget_high_u16(ini1);		//split
				ini1_low = vget_low_u16(ini1);
				ini1_high_long = vmovl_u16(ini1_high);	//to uint_32x4_t
				ini1_low_long = vmovl_u16(ini1_low);
				ini1_high_f = vcvtq_f32_u32(ini1_high_long); //to float32x4_t
				ini1_low_f = vcvtq_f32_u32(ini1_low_long);
				//load vector muy
				muy_low_v = vld1q_f32(muy_p);
				muy_p+=4;
				muy_high_v = vld1q_f32(muy_p);
				//calculate muy initials
				muy_high_v = vaddq_f32(muy_high_v, ini1_high_f);
				muy_low_v = vaddq_f32(muy_low_v, ini1_low_f);
				if(m==4){
					float32x4_t frame_num = vdupq_n_f32(0.2);
					muy_high_v = vmulq_f32(muy_high_v,frame_num);
					muy_low_v = vmulq_f32(muy_low_v,frame_num);
				}
				//store muy
				muy_p-=4;
				vst1q_f32(muy_p, muy_low_v);
				muy_p+=4;
				vst1q_f32(muy_p, muy_high_v);
				muy_p+=4;
			}
		}
		muy_p = muy;
	}
	//calculate variance
	frame_ini_p = frame_ini;
	for(int m=0;m<5;m++){
		for(int i=0;i<480;i++){
			for(int j=0;j<80;j++){
				//load pixel value
				ini = vld1_u8(frame_ini_p);
				frame_ini_p+=8;
				//convert to float
				ini1 = vmovl_u8(ini);
				ini1_high = vget_high_u16(ini1);		//split
				ini1_low = vget_low_u16(ini1);
				ini1_high_long = vmovl_u16(ini1_high);	//to uint_32x4_t
				ini1_low_long = vmovl_u16(ini1_low);
				ini1_high_f = vcvtq_f32_u32(ini1_high_long); //to float32x4_t
				ini1_low_f = vcvtq_f32_u32(ini1_low_long);
				//load vector muy
				muy_low_v = vld1q_f32(muy_p);
				muy_p+=4;
				muy_high_v = vld1q_f32(muy_p);
				muy_p+=4;
				//load vector var
				var_low_v = vld1q_f32(var_p);
				var_p+=4;
				var_high_v = vld1q_f32(var_p);
				//calculate variance_ini: variance = variance + (muy-I)^2
				dif_high = vabdq_f32(ini1_high_f, muy_high_v);
				dif_low = vabdq_f32(ini1_low_f, muy_low_v);
				dif_high = vmulq_f32(dif_high, dif_high);
				dif_low = vmulq_f32(dif_low, dif_low);
				var_high_v = vaddq_f32(var_high_v, dif_high);
				var_low_v = vaddq_f32(var_low_v, dif_low);
				if(m==4){
					float32x4_t frame_num = vdupq_n_f32(0.2);
					var_high_v = vmulq_f32(var_high_v,frame_num);
					var_low_v = vmulq_f32(var_low_v,frame_num);
				}
				//store var
				var_p-=4;
				vst1q_f32(var_p, var_low_v);
				var_p+=4;
				vst1q_f32(var_p, var_high_v);
				var_p+=4;
			}
		}
		muy_p = muy;
		var_p = var;
	}
}
//output density
//uint32_t findDensity(uint8_t *frame, float32_t *muy, float32_t *var, uint8_t *fg){
uint32_t findDensity(uint8_t *frame, float32_t *muy, float32_t *var){
	uint32_t cnt_gauss[4]={0,0,0,0};
	uint32x4_t count_fg = vdupq_n_u32(0);
	uint8_t *frame_p = frame;
	float32x4_t dif_high, dif_low;
	float32_t *muy_p = muy;
	float32x4_t muy_high_v, muy_low_v;
	float32_t *var_p = var;
	float32x4_t var_high_v, var_low_v;
	//uint8_t *fg_p = fg;
	for(int i=0;i<480;i++){
		for(int j=0;j<80;j++){
			//load pixel value
			ini = vld1_u8(frame_p);
			frame_p+=8;
			//convert to float
			ini1 = vmovl_u8(ini);
			ini1_high = vget_high_u16(ini1);		//split
			ini1_low = vget_low_u16(ini1);
			ini1_high_long = vmovl_u16(ini1_high);	//to uint_32x4_t
			ini1_low_long = vmovl_u16(ini1_low);
			ini1_high_f = vcvtq_f32_u32(ini1_high_long); //to float32x4_t
			ini1_low_f = vcvtq_f32_u32(ini1_low_long);
			//load vector muy
			muy_low_v = vld1q_f32(muy_p);
			muy_p+=4;
			muy_high_v = vld1q_f32(muy_p);
			//load vector var
			var_low_v = vld1q_f32(var_p);
			var_p+=4;
			var_high_v = vld1q_f32(var_p);

			//calculate muy = alpha*I+(1-alpha)*muy
				//low part
			muy_low_v = vmulq_n_f32(muy_low_v, 1-alpha);			//alpha*I
			muy_low_v = vmlaq_n_f32(muy_low_v, ini1_low_f, alpha); //alpha*I+(1-alpha)*muy

				//high part
			muy_high_v = vmulq_n_f32(muy_high_v, 1-alpha);		//alpha*I
			muy_high_v = vmlaq_n_f32(muy_high_v, ini1_high_f, alpha); //alpha*I+(1-alpha)*muy

				//store muy
			muy_p-=4;
			vst1q_f32(muy_p, muy_low_v);
			muy_p+=4;
			vst1q_f32(muy_p, muy_high_v);
			muy_p+=4;

			//calculate var: variance = alpha*d*d + (1-alpha)*variance
				//low part
			dif_low = vabdq_f32(ini1_low_f, muy_low_v);
			var_low_v = vmulq_n_f32(var_low_v, 1-alpha);
			var_low_v = vmlaq_n_f32(var_low_v, dif_low, alpha);
				//high part
			dif_high = vabdq_f32(ini1_high_f, muy_high_v);
			var_high_v = vmulq_n_f32(var_high_v, 1-alpha);
			var_high_v = vmlaq_n_f32(var_high_v, dif_high, alpha);
				//store var
			var_p-=4;
			vst1q_f32(var_p, var_low_v);
			var_p+=4;
			vst1q_f32(var_p, var_high_v);
			var_p+=4;

			//compare foreground when |I-muy| > k*delta --> M=1
			float32x4_t thresh;
				//low part
			thresh = sqrtv(var_low_v);
			thresh = vmulq_n_f32(thresh, k);
			uint32x4_t foreground_low = vcgtq_f32(dif_low, thresh);		//if yes, 32 bit is 1
			foreground_low = vshrq_n_u32(foreground_low, 31);	//shift right 31 to remain only one 1bit
				//high part
			thresh = sqrtv(var_high_v);
			thresh = vmulq_n_f32(thresh, k);
			uint32x4_t foreground_high = vcgtq_f32(dif_high, thresh);
			foreground_high = vshrq_n_u32(foreground_high, 31);	//shift right 31 to remain only one 1bit

			count_fg = vld1q_u32(cnt_gauss);
			count_fg = vaddq_u32(count_fg, foreground_low);
			count_fg = vaddq_u32(count_fg, foreground_high);
			//store count of gaussian foreground
			uint32_t *cnt_gauss_p = cnt_gauss;
			vst1q_u32(cnt_gauss_p, count_fg);

			/*//combine and store to foreground
			uint16x4_t foreground_low1 = vmovn_u32(foreground_low);
			uint16x4_t foreground_high1 = vmovn_u32(foreground_high);
			uint16x8_t foreground_com = vcombine_u16(foreground_low1, foreground_high1);
			uint8x8_t fg1 = vmovn_u16(foreground_com);
			vst1_u8(fg_p, fg1);
			fg_p+=8;*/
		}
	}
	uint32_t cnt_gauss_total = cnt_gauss[0]+cnt_gauss[1]+cnt_gauss[2]+cnt_gauss[3];
	uint32_t density = cnt_gauss_total/3072;
	return density;
}


uint32_t findDensity_test(uint8_t *frame, float32_t *muy, float32_t *var, uint8_t *fg){
    uint32_t cnt_gauss[4]={0,0,0,0};
	uint32x4_t count_fg = vdupq_n_u32(0);
	uint8_t *frame_p = frame;
	float32x4_t dif_high, dif_low;
	float32_t *muy_p = muy;
	float32x4_t muy_high_v, muy_low_v;
	float32_t *var_p = var;
	float32x4_t var_high_v, var_low_v;
	uint8_t *fg_p = fg;
	for(int i=0;i<480;i++){
		for(int j=0;j<80;j++){
			//load pixel value
			ini = vld1_u8(frame_p);
			frame_p+=8;
			//convert to float
			ini1 = vmovl_u8(ini);
			ini1_high = vget_high_u16(ini1);		//split
			ini1_low = vget_low_u16(ini1);
			ini1_high_long = vmovl_u16(ini1_high);	//to uint_32x4_t
			ini1_low_long = vmovl_u16(ini1_low);
			ini1_high_f = vcvtq_f32_u32(ini1_high_long); //to float32x4_t
			ini1_low_f = vcvtq_f32_u32(ini1_low_long);
			//load vector muy
			muy_low_v = vld1q_f32(muy_p);
			muy_p+=4;
			muy_high_v = vld1q_f32(muy_p);
			//load vector var
			var_low_v = vld1q_f32(var_p);
			var_p+=4;
			var_high_v = vld1q_f32(var_p);

			//calculate muy = alpha*I+(1-alpha)*muy
				//low part
			muy_low_v = vmulq_n_f32(muy_low_v, 1-alpha);			//alpha*I
			muy_low_v = vmlaq_n_f32(muy_low_v, ini1_low_f, alpha); //alpha*I+(1-alpha)*muy

				//high part
			muy_high_v = vmulq_n_f32(muy_high_v, 1-alpha);		//alpha*I
			muy_high_v = vmlaq_n_f32(muy_high_v, ini1_high_f, alpha); //alpha*I+(1-alpha)*muy

				//store muy
			muy_p-=4;
			vst1q_f32(muy_p, muy_low_v);
			muy_p+=4;
			vst1q_f32(muy_p, muy_high_v);
			muy_p+=4;

			//calculate var: variance = alpha*d*d + (1-alpha)*variance
				//low part
			dif_low = vabdq_f32(ini1_low_f, muy_low_v);
			var_low_v = vmulq_n_f32(var_low_v, 1-alpha);
			var_low_v = vmlaq_n_f32(var_low_v, dif_low, alpha);
				//high part
			dif_high = vabdq_f32(ini1_high_f, muy_high_v);
			var_high_v = vmulq_n_f32(var_high_v, 1-alpha);
			var_high_v = vmlaq_n_f32(var_high_v, dif_high, alpha);
				//store var
			var_p-=4;
			vst1q_f32(var_p, var_low_v);
			var_p+=4;
			vst1q_f32(var_p, var_high_v);
			var_p+=4;

			//compare foreground when |I-muy| > k*delta --> M=1
			float32x4_t thresh;
				//low part
			thresh = sqrtv(var_low_v);
			thresh = vmulq_n_f32(thresh, k);
			uint32x4_t foreground_low = vcgtq_f32(dif_low, thresh);		//if yes, 32 bit is 1
			foreground_low = vshrq_n_u32(foreground_low, 31);	//shift right 31 to remain only one 1bit
				//high part
			thresh = sqrtv(var_high_v);
			thresh = vmulq_n_f32(thresh, k);
			uint32x4_t foreground_high = vcgtq_f32(dif_high, thresh);
			foreground_high = vshrq_n_u32(foreground_high, 31);	//shift right 31 to remain only one 1bit

			count_fg = vld1q_u32(cnt_gauss);
			count_fg = vaddq_u32(count_fg, foreground_low);
			count_fg = vaddq_u32(count_fg, foreground_high);
			//store count of gaussian foreground
			uint32_t *cnt_gauss_p = cnt_gauss;
			vst1q_u32(cnt_gauss_p, count_fg);

			//combine and store to foreground
			uint16x4_t foreground_low1 = vmovn_u32(foreground_low);
			uint16x4_t foreground_high1 = vmovn_u32(foreground_high);
			uint16x8_t foreground_com = vcombine_u16(foreground_low1, foreground_high1);
			uint8x8_t fg1 = vmovn_u16(foreground_com);
			vst1_u8(fg_p, fg1);
			fg_p+=8;
		}
	}
	uint32_t cnt_gauss_total = cnt_gauss[0]+cnt_gauss[1]+cnt_gauss[2]+cnt_gauss[3];
	uint32_t density = cnt_gauss_total/3072;
	return density;
}
