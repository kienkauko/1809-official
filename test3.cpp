 #include <opencv2/core/core.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <opencv2/features2d/features2d.hpp>
 #include <opencv2/opencv.hpp>
 #include <opencv/cv.h>

 #include <time.h>
 #include <iostream>
 #include <cmath>
 #include <vector>
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <assert.h>
 #include <getopt.h>             /* getopt_long() */
 #include <fcntl.h>              /* low-level i/o */
 #include <unistd.h>
 #include <errno.h>
 #include <sys/stat.h>
 #include <sys/types.h>
 #include <sys/time.h>
 #include <sys/ioctl.h>
 #include <stdint.h>
 #include <sys/mman.h>
 #include <sys/socket.h>
 #include <netinet/in.h>
 #include <arpa/inet.h>
 #include <linux/videodev2.h>
 #include <arm_neon.h>
 #include <pthread.h>

 #include "data_type.hpp"
 #include "jpeg_decode.hpp"
 #include "rgb2gray.hpp"
 #include "density.hpp"

#define BILLION 1000000000L

#define IMG_TEMP "anh_temp.png"

#define ID_CAMERA "5"

using namespace std;
using namespace cv;

/*
*	Thuc hien do bang so SO_LAN_DO
*
* 	Thong so can gui ve server:
*		av_density: (int) lay trung binh cong
*/
#define SO_LAN_DO 10

int av_density_arr[SO_LAN_DO];
int lan_do = 0;


float d=3;								// Do dai canh tam giac calib

uint8_t gray5f[5*307200];								// 5 frames for
uint8_t red[307200], green[307200], blue[307200];		// 307200 = 640x480
uint8_t gray_a[5][307200], gray_a2[5][307200];        	// pyramid level fixed
uint8_t edge_en_im[157136];								// 157136 = 488x322
int32_t Sx2[76032];										// 200x384
int32_t Sy2[76032];
int32_t Sxy[76032];
float32_t muy[307200], var[307200];
uint8_t bin[307200];                         // for density test only

uint32_t av_density;

/*		Su dung cho phan dieu khien Camera 	*/
/*************************************************/
bool ctrl_continue = true;
int fd_cam_main;
struct v4l2_buffer buf_v4l2;
fd_set fds_main;
struct timeval tv_main = {0};
int r;
/*************************************************/

//unsigned char buf_vel[buf_v4l2.length];
unsigned char buf_vel[921600];	//// buf_v4l2.length : 640*480*3 = 921600
unsigned char *buffer;
int frame_count = 0;
int network_socket;
struct timespec start_send, end_send;
void decode_gray(){
	jpeg_decode(buf_vel, red, green, blue);
    rgbtogray (red, green, blue, gray_a[0]);
	jpeg_decode(buffer, red, green, blue);
	rgbtogray (red, green, blue, gray_a2[0]);
}

void density(){
	av_density = findDensity_test(gray_a[0], muy, var, bin);
	av_density_arr[lan_do] = av_density; //KIEN FIX

	  
    	frame_count++;
	cout << "Frame: " <<frame_count << endl;
//        cout << '-' << "Time diff: " << time_dif << endl;
    	cout << '-' << "Density: " << av_density << '%' << endl;
}

void send_density()
{
	printf("Send to server..\n");
	av_density = 0;
	for (int i = 0; i < SO_LAN_DO; ++i)
	{
		av_density += av_density_arr[i];
	}
	av_density = av_density / SO_LAN_DO;
	lan_do = 0;
	

/*..............write image into client device ................*/
	unsigned char buf_tuyen[921600];
	for(int i=0; i<307200;i++){
		unsigned int temp = i*3;
		buf_tuyen[temp] = red[i] ;
		buf_tuyen[temp+1] = green[i];
		buf_tuyen[temp+2] = blue[i];
	}
	
	Mat freshFrame = Mat(480,640, CV_8UC3, buf_tuyen);

	vector<int> compression_params;
	compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);

	// Co the thay doi do nen cua anh, tuy thuoc vao dieu kien duong
	compression_params.push_back(20);
	
//	imwrite("anh_temp.png",freshFrame,compression_params);
	imwrite(IMG_TEMP,freshFrame,compression_params);
	/*..............end of write image into client device ................*/

	//printf("Ok lan 6\n");
	cout << "final density:" << av_density << endl;// kien sua /////
	int r = send(network_socket, &av_density, sizeof(av_density),0);
	if (r < 0) printf("Error send density\n");
}

int main(){
    //Khoi tao socket 
	int connfd = 0, err;
	char sendBuff[256];
//	int network_socket;
	network_socket = socket(AF_INET, SOCK_STREAM, 0);
	if (network_socket < 0) printf("Error when create socket\n");
	else printf("Successful create socket\n");
	// specify an address for the socket
	struct sockaddr_in server_address;
	server_address.sin_family = AF_INET;
	server_address.sin_port = htons(9002);
	 //If want to handle the IP address 
	// get ip 
	char ip[50];
	printf("Enter IP address to connect: ");
    	gets(ip);
	server_address.sin_addr.s_addr = inet_addr(ip);
	int connection_status = connect(network_socket, (struct sockaddr *) &server_address, sizeof(server_address));
	if (connection_status == -1)
	{
		printf("There was an error making a connection to the remote socket \n \n ");}
	//server_address.sin_addr.s_addr = INADDR_ANY;

	//--------------------------------------------------------------
    	char text[255];
    	int32_t *Sx2_p = Sx2;
    	int32_t *Sy2_p = Sy2;
    	int32_t *Sxy_p = Sxy;
    	int lf = 200;
    	Mat binary(480,640,CV_8U);
	Mat frame(480,640,CV_8U);
	Mat frame_c, frame2_c;
	Mat frame2(480,640,CV_8U);

	// Su dung timer khi truyen anh loi
	// Dinh nghia o file tcp_ip.cpp
//	init_timer();
	
	//vector<point_int> corner, track_corner;
	//point_f a_img, b_img, c_img;
	//float h_b_phi[3];
	//float b_exp[4], sin_exp[4], cos_exp[4];

	uint8_t *gray5f_p = gray5f;
	//float time_dif;
	struct timespec	time1, time2, time3;



	//----------------------------- Cam Init ------------------------------
	// Open a descriptor to the device
	//tuyencmt
	

    if ((fd_cam_main = open("/dev/video0", O_RDWR)) < 0){ // camera is detected with value >0
		perror("open");
		exit(1);
	}
	else
		cout << "\n Open _dev_video0: OK";

	// Retrieve the device’s capabilities
	struct v4l2_capability cap;
    if (ioctl(fd_cam_main, VIDIOC_QUERYCAP, &cap) < 0) {
		perror("VIDIOC_QUERYCAP");
		exit(1);
    }
    else
    	cout << "\n VIDIOC_QUERYCAP: Ok";

	if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
		fprintf(stderr, "The device does not handle single-planar video capture.\n");
		exit(1);
	}
	else
		cout << "\nThe device does not handle single-planar video capture.";

	// Set our video format
	struct v4l2_format fmt;
	fmt.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
	//fmt.fmt.pix.field = V4L2_FIELD_ANY;
	fmt.fmt.pix.width       = 640;
    fmt.fmt.pix.height      = 480;

	if(ioctl(fd_cam_main, VIDIOC_S_FMT, &fmt) < 0){
		perror("VIDIOC_S_FMT");
		exit(1);
	}
	else
		cout << "\nVIDIOC_S_FMT: OK";

	// Inform the device about your future buffers
    struct v4l2_requestbuffers req = {0};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == ioctl(fd_cam_main, VIDIOC_REQBUFS, &req))
    {
        perror("Requesting Buffer");
        return 1;
    }
    else
    	cout << "\n Requesting Buffer: OK";

	// Allocate your buffers
	//tuyencmt
	//struct v4l2_buffer buf_v4l2;

	memset(&buf_v4l2, 0, sizeof(buf_v4l2));
    buf_v4l2.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf_v4l2.memory = V4L2_MEMORY_MMAP;
    buf_v4l2.index = 0;

    if(ioctl(fd_cam_main, VIDIOC_QUERYBUF, &buf_v4l2) < 0){
        perror("VIDIOC_QUERYBUF");
        exit(1);
    }

// for decode function
//    unsigned char *
	buffer = (unsigned char*)mmap(NULL, buf_v4l2.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_cam_main, buf_v4l2.m.offset);

    // buf_v4l2.length : 640*480*3 = 921600
    if(buffer == MAP_FAILED){
        perror("mmap");
        exit(1);
    }
    else
    	cout << "\nmmap: OK";
    
    if (-1 == ioctl(fd_cam_main, VIDIOC_STREAMON, &buf_v4l2.type)) {
        perror("Start Capture");
        return 1;
    }
    else
    	cout << "\n Starting Capture...\n";
	// -------------------------- Discard first frame ------------------------
	for(int i=0; i<10; i++){
		if (-1 == ioctl(fd_cam_main, VIDIOC_QBUF, &buf_v4l2)) {
            		perror("Query Buffer");
            		return 1;
        	}

		FD_ZERO(&fds_main); // create empty file 
		FD_SET(fd_cam_main, &fds_main); // copy fd_cam_main into fds_main 
		tv_main.tv_sec = 2; //assign 2 second 
		r = select(fd_cam_main + 1, &fds_main, NULL, NULL, &tv_main); // check operating camera from range video0 to video[fd_cam_main] 		within 2 seconds
		if (-1 == r) {
			perror("Waiting for Frame");
			return 1;
		}

        	if (-1 == ioctl(fd_cam_main, VIDIOC_DQBUF, &buf_v4l2)) {
            		perror("Retrieving Frame");
            		return 1;
        	}
	}
	
	/*----------------------------------------------------------------*/
	/*---------------End TESTING CAMERA-------------------------------*/
	
	
	/*-----------------------------------------------------------------*/
	/* --------Begin encode and transform to grey image ---------------*/
	// -------------------------- Brackground init -----------------------
    for(int i=0;i<5;i++){
		if (-1 == ioctl(fd_cam_main, VIDIOC_QBUF, &buf_v4l2)) {
            perror("Query Buffer");
            return 1;
        }

		FD_ZERO(&fds_main);
		FD_SET(fd_cam_main, &fds_main);
		tv_main.tv_sec = 2;
		r = select(fd_cam_main + 1, &fds_main, NULL, NULL, &tv_main);
		if (-1 == r) {
			perror("Waiting for Frame");
			return 1;
		}

        if (-1 == ioctl(fd_cam_main, VIDIOC_DQBUF, &buf_v4l2)) {
            perror("Retrieving Frame");
            return 1;
        }

		jpeg_decode(buffer, red, green, blue);
        rgbtogray (red, green, blue, gray5f_p);
        gray5f_p+=307200;
    }
	gray5f_p = gray5f;
    background_initiate(gray5f_p, muy, var);
//	int frame_count = 0;
// for decode fucntion
//	unsigned char buf_vel[buf_v4l2.length];
	/*-----------------------------------------------------------------*/
	/*----------------------Bat dau vong while ------------------------*/
	while(1){
		printf("Dau vong while\n");
		// --------------------- Get 2 consecutive frames --------------------
		if (-1 == ioctl(fd_cam_main, VIDIOC_QBUF, &buf_v4l2)) {
            perror("Query Buffer");
            return 1;
        }

		FD_ZERO(&fds_main);
		FD_SET(fd_cam_main, &fds_main);
		tv_main.tv_sec = 2;
		r = select(fd_cam_main + 1, &fds_main, NULL, NULL, &tv_main);
		if (-1 == r) {
			perror("Waiting for Frame");
			return 1;
		}

        if (-1 == ioctl(fd_cam_main, VIDIOC_DQBUF, &buf_v4l2)) {
            perror("Retrieving Frame");
            return 1;
        }
		clock_gettime(CLOCK_MONOTONIC, &time1);			// Get time 1

		memcpy(buf_vel, buffer, buf_v4l2.length);

		if (-1 == ioctl(fd_cam_main, VIDIOC_QBUF, &buf_v4l2)) {
            perror("Query Buffer");
            return 1;
        }

		FD_ZERO(&fds_main);
		FD_SET(fd_cam_main, &fds_main);
		tv_main.tv_sec = 2;
		r = select(fd_cam_main + 1, &fds_main, NULL, NULL, &tv_main);
		if (-1 == r) {
			perror("Waiting for Frame");
			return 1;
		}

        if (-1 == ioctl(fd_cam_main, VIDIOC_DQBUF, &buf_v4l2)) {
            perror("Retrieving Frame");
            return 1;
        }

		clock_gettime(CLOCK_MONOTONIC, &time2);			// Get time 2
		
		// ------------------------ Decode and convert to gray ----------------------

		decode_gray();
		
		// ---------------------- DENSITY ESTIMATION ---------------
		density();
 		
/************************************************************************************
*	Phan code The anh them vao de gui anh len server
*	Create: 26.04.2018
*	Last modify:
*
*************************************************************************************
*/
		++lan_do;
		if (lan_do < SO_LAN_DO ) continue;
		//usleep(230000);
		clock_gettime(CLOCK_REALTIME, &start_send);
		/*----------------------------------------------------------------*/
		time_t t_begin,t_end;
		send_density();
		clock_gettime(CLOCK_REALTIME, &end_send);
		double difference = (end_send.tv_sec - start_send.tv_sec) + (double)(end_send.tv_nsec - start_send.tv_nsec)/1000000000.0d;
    	cout << "It took " << difference << " seconds to send " << endl;

		}
	/*---------------------------------------------------------------------*/
	/*--------------------------End while----------------------------------*/
	close(network_socket);
	return 0;
}
