all:
	g++ -std=c++11 `pkg-config --cflags opencv` density.cpp jpeg_decode.cpp rgb2gray.cpp test1.cpp -o test1 -ljpeg `pkg-config --cflags --libs opencv`
compile:
