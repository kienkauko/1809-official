#include "jpeg_decode.hpp"

void jpeg_decode(unsigned char *buffer, uint8_t red[307200], uint8_t green[307200], uint8_t blue[307200]){

	unsigned long data_size;     // length of the buffer
	data_size = 640 * 480 * 3;
	unsigned char *rowptr[1];    // pointer to an array
	unsigned char *jdata = new unsigned char[data_size];       // data for the image

	struct jpeg_decompress_struct info; //for our jpeg info
	struct jpeg_error_mgr err;          //the error handler

	info.err = jpeg_std_error(& err);
	jpeg_create_decompress(& info);   //fills info structure

	jpeg_mem_src(&info, buffer, data_size);
	jpeg_read_header(&info, TRUE);   // read jpeg file header

	jpeg_start_decompress(&info);    // decompress the file


	//--------------------------------------------
	// read scanlines one at a time & put bytes
	//    in jdata[] array. Assumes an RGB image
	//--------------------------------------------
	while (info.output_scanline < info.output_height) // loop
	{
	// Enable jpeg_read_scanlines() to fill our jdata array
	rowptr[0] = (unsigned char *)jdata +  // secret to method
			3* info.output_width * info.output_scanline;

	jpeg_read_scanlines(&info, rowptr, 1);
	}
	//---------------------------------------------------

	jpeg_finish_decompress(&info);   //finish decompressing
	jpeg_destroy_decompress(&info);

	for(int i=0; i<307200;i++){
	unsigned int temp = i*3;
		blue[i] =*(jdata+temp);
		green[i] =*(jdata+temp+1);
		red[i] =*(jdata+temp+2);
	}
	delete[] jdata;
}
