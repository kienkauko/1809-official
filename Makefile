all:
	g++ `pkg-config --cflags opencv` density.cpp jpeg_decode.cpp rgb2gray.cpp test1.cpp -o test1 -mfpu=neon -ljpeg `pkg-config --libs opencv`
compile:
