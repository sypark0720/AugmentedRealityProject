#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>  //Image file reading and writing
#include <opencv2/imgproc.hpp>   //Image processing
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "Config.h"
#include "Tracker.h"

using namespace cv;
using namespace std;

int main()
{
	Tracker tracker;
	Mat img_raw_input;
	Mat img_input, img_out;

	VideoCapture cap;

	cap.open(1);
	//use 640X480
	cap.set(CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CAP_PROP_FRAME_HEIGHT, 720);

	if (!cap.isOpened())
	{
		cout << "Error: cannot open the webcam" << endl;
		return -1;
	}

	//--- GRAB AND WRITE LOOP
	cout << "Start grabbing" << endl
		<< "Press any key to terminate" << endl;

	int i = 0;
	while (true)
	{
		i++;

		//print the frame number
		int fps = cap.get(CAP_PROP_FPS);
		//cout << fps << " " << i << endl;

		////юс╫ц: read the input image
		//img_raw_input = imread("E:\\Workspace\\Visual Studio\\opencvTest\\MarkerDetectionTest\\capture.png", 1);
		//resize(img_raw_input, img_input, Size(320, 240));

		//read the input image
		bool ret = cap.read(img_raw_input);
		//cout << "width:" << img_input.size().width << endl;
		//cout << "height:" << img_input.size().height << endl;


		double m[] = { fx, 0, cx, 0, fy, cy, 0, 0, 1 };
		Mat camIntrinsic = Mat(3, 3, CV_64FC1, m);

		double d[] = { k1, k2, p1, p2 };
		Mat distCoeffs = Mat(4, 1, CV_64FC1, d);

		undistort(img_raw_input, img_input, camIntrinsic, distCoeffs);

		//img_input.copyTo(img_out);

		if (!ret)
		{
			cout << "Error: cannot read images from the webcam" << endl;
			break;
		}

		if (i > 15) {
			imshow("read", img_input);
			waitKey(3);
			tracker.processFrame(img_input, img_out);
			imshow("out", img_out);
			waitKey(3);
		}
		// END process
		if (waitKey(5) >= 0)
			break;
	}

	return 0;
}