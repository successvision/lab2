#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>
#include <iostream>
#include <ctype.h>


using namespace cv;
using namespace std;


#define MOVIE 1
#define CAM 0
#define SPLIT 1
#define UART 1
#if UART
//-------------------------
//----- SETUP USART 0 -----
//-------------------------
//At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
int uart0_filestream = -1;

//OPEN THE UART
//The flags (defined in fcntl.h):
//	Access modes (use 1 of these):
//		O_RDONLY - Open for reading only.
//		O_RDWR - Open for reading and writing.
//		O_WRONLY - Open for writing only.
//
//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
//											immediately with a failure status if the output can't be written immediately.
//
//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
if (uart0_filestream == -1)
{
	//ERROR - CAN'T OPEN SERIAL PORT
	printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
}

//CONFIGURE THE UART
//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
//	CSIZE:- CS5, CS6, CS7, CS8
//	CLOCAL - Ignore modem status lines
//	CREAD - Enable receiver
//	IGNPAR = Ignore characters with parity errors
//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
//	PARENB - Parity enable
//	PARODD - Odd parity (else evena)
struct termios options;
tcgetattr(uart0_filestream, &options);
options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
options.c_iflag = IGNPAR;
options.c_oflag = 0;
options.c_lflag = 0;
tcflush(uart0_filestream, TCIFLUSH);
tcsetattr(uart0_filestream, TCSANOW, &options);


#endif
Mat image;
bool selectObject = false;
int trackObject = 0;
Point origin;
Rect selection;
int ymin = 251, ymax = 253, Crmin = 127, Crmax = 129, Cbmin = 127, Cbmax = 129;




/// On mouse event 
static void onMouse(int event, int x, int y, int, void*)
{
	if (selectObject)  // for any mouse motion
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);
		selection &= Rect(0, 0, image.cols, image.rows);  /// Bitwise AND  check selectin is within the image coordinate
	}

	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		origin = Point(x, y);
		selection = Rect(x, y, 0, 0);
		selectObject = true;
		cout << origin << endl;
		break;

	case CV_EVENT_LBUTTONUP:
		selectObject = false;
		if (selection.width > 0 && selection.height > 0)
		{
			trackObject = -1;
		}
		break;
	}
}


int main()
{

	Mat YCbCr, hsv, hue, mask, dst, dst_morp, morp_src;

	Mat frame;
	int camNum = 0;
	int pre_center_x = 0, center_x = 0;
	int pre_center_y = 0, center_y = 0;
	int firstRun = 0;
	int diff_x = 0;
	int diff_y = 0;
	int count = 0;
#if MOVIE
	VideoCapture cap;
	VideoCapture capture;
	//capture.open("RoadVideo2.avi");
	//capture.open("soccer1.avi");
	capture.open("SunTravelling.avi");
	if (!capture.isOpened()){
		cout << " Video not read \n";
		return 1;
	}
	//int rate=capture.get(CV_CAP_PROP_FPS);
	capture.read(frame);
#endif MOVIE
	//----- TX BYTES -----
	unsigned char tx_buffer[20];
	unsigned char *p_tx_buffer;

	p_tx_buffer = &tx_buffer[0];
	*p_tx_buffer++ = 'H';
	*p_tx_buffer++ = 'e';
	*p_tx_buffer++ = 'l';
	*p_tx_buffer++ = 'l';
	*p_tx_buffer++ = 'o';

	if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write
		if (count < 0)
		{
			printf("UART TX error\n");
		}
}
#if CAM
	VideoCapture cap;
	//VideoCapture capture;   
	cap.open(camNum);
	if (!cap.isOpened())
	{
		cout << "***Could not initialize capturing...***\n";
		cout << "Current parameter's value: \n";
		waitKey(0);
	}
	cout << "Camera Read OK \n";
	cap >> frame;  // Read from cam

#endif CAM
	int isempty = 1;


	// TrackBar 설정
	namedWindow("Source", 0);
	setMouseCallback("Source", onMouse, 0);

	while (isempty)
	{

#if MOVIE
		if (!capture.read(frame))
			break;
		frame.copyTo(image);
#endif MOVIE

#if CAM
		cap >> frame;  // Read from cam
		if (frame.empty())
			break;
		frame.copyTo(image);
#endif CAM      
		cvtColor(image, YCbCr, CV_BGR2YCrCb);
#if SPLIT

		vector<Mat> bgr, hsvs, ycc;
		split(image, bgr);
		split(hsv, hsvs);
		split(YCbCr, ycc);
		// - LAB   
#endif SPLIT

		/// set dst as the output of InRange
		inRange(YCbCr, Scalar(MIN(ymin, ymax), MIN(Crmin, Crmax), MIN(Cbmin, Cbmax)), Scalar(MAX(ymin, ymax), MAX(Crmin, Crmax), MAX(Cbmin, Cbmax)), dst);

		int n1 = cv::countNonZero(dst);

		Point a(50, 50);
		if (n1 == 0)
			dst.at<uchar>(a) = 1;

		namedWindow("InRange", 0);
		imshow("InRange", dst);

		GaussianBlur(dst, dst, Size(3, 3), 0, 0, 4);

		////  Draw Contour   ////
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		Mat dst2;
		dst.copyTo(dst2);
		findContours(dst2, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		if (!contours.empty()){


			// iterate through all the top-level contours,
			// draw each connected component with its own random color
			int idx = 0, largestComp = 0;
			double maxArea = 0;

			for (; idx >= 0; idx = hierarchy[idx][0])
			{
				const vector<Point>& c = contours[idx];
				double area = fabs(contourArea(Mat(c))); //contourArea(contours[idx]),
				if (area > maxArea)
				{
					maxArea = area;
					largestComp = idx;
				}
			}

			drawContours(dst2, contours, largestComp, Scalar(255, 0, 255), 2, 8, hierarchy);
			float radius = 0;


			Rect r0 = boundingRect(Mat(contours[largestComp]));

			rectangle(dst2, r0, Scalar(255, 0, 0), 2);
			rectangle(image, r0, Scalar(0, 255, 0), 2);  //Draw rectangle in image


			Point center(r0.x + r0.width / 2, r0.y + r0.height / 2);
			circle(image, center, 1, Scalar(255, 0, 0), 5);

			string text = format("Center = %d %d", center);
			putText(image, text, center, FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);

			/// Draw Dst
			namedWindow("Contour", 0);
			imshow("Contour", dst2);
			imshow("Source", image);

			///////
			if (!firstRun){
				pre_center_x = center.x;
				pre_center_y = center.y;
				firstRun++;
			}
			else if (count == 10){
				center_x = center.x;
				center_y = center.y;
				diff_x = center_x - pre_center_x;
				diff_y = center_y - pre_center_y;
				pre_center_x = center_x;
				pre_center_y = center_y;
				cout << "DX=" << diff_x << endl;
				cout << "DY=" << diff_y << endl;
				count = 0;
			}   //0.1초마다 데이터 갱신
			count++;
			// Options
			char c = (char)waitKey(10);    // sample_rate 100Hz
			if (c == 27)
				break;
		}
		else{
			isempty = 0;
		}
	}
	cout << "end" << endl;
	// end of while
	return 0;

}