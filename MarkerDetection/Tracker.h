#pragma once
#include <opencv2/core/mat.hpp>  
#include <opencv2/imgcodecs.hpp>  //Image file reading and writing
#include <opencv2/imgproc.hpp>   //Image processing
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "Config.h"

using namespace cv;
using namespace std;

class Tracker
{
public:

	// sorter: sort points by x value in ascent order
	struct markerSquareSorter
	{
		bool operator () (Point2f a, Point2f b) {
			if (a.x == b.x)
				return a.y < b.y;
			return a.x < b.x;
		}
	} markerSquareSorter;

	void processFrame(Mat &img, Mat &out);

	Tracker();
	~Tracker();

private:

	Mat img_out;
	Mat img_hsv;
	Mat img_binary;

	vector<vector<Point>> contours;
	vector<vector<Point2f>> markerPoints;
	vector<Point2f> cornerPoints;
	vector<int> cornerIdx; 
	vector <vector<Point2f>> groupedPoints;
	vector <vector<int>> groupedMarkerIndex;
	vector<vector<float>> widthHeightOfGroup;

	//4.5. markerImgSquarePoints
	map<int, vector<Point2f>> markerImgSquarePoints;

	//5. Camera Pose
	//3d points for each marker square (30 X 30)
	vector<Point3f> markerSquarePoints3d;

	Mat camIntrinsic;
	Mat distCoeffs;
	map<int, Mat> cameraRvecs;
	map<int, Mat> cameraTvecs;

	vector<Mat> cameraRvecsForGroup;
	vector<Mat> cameraTvecsForGroup;

	//Drawing
	vector<Point3f> cubeBottomPoints;
	vector<Point3f> cubeTopPoints;

	void endProcessFrame();

	// <Functions for each step>
	// 1. Binarize the image
	void binarizeImg(const Mat &img, Mat & out);

	// 3. Process each contours:각 contour마다 6 points 저장 & corner point를 찾는다.
	void processContours(const vector<Point> contour);
	void findTwoMaxEdges(vector<Point2f> approx, int firstEdgePoints[], int secondEdgePoints[]);
	int findOverlappedPoint(int array1[], int array2[]);

	// 4. Grouping
	void grouping();
	void deleteUncompletedGroup();
	void sortEachGroup();
	//void sortEachCornerGroup();
	Point2f getConerRightPoint(int markerIdx);
	Point2f getCenterPoint(int markerIdx);

	//4.5. Get square points for each marker // BottomLeft가 첫 번째, 그 뒤로 반시계 방향
	vector<Point2f> findImgSquarePointsOfMarker(int markerIdx);

	//5. Get Camera Pose
	
	void getCameraPosForGroups();

	//6. Drawing 
	void drawByMarkerPos();
	void drawByGroupPos();

	//Util
	float euclideanD(Point2f point1, Point2f point2);
	double euclidean3D(Mat point1, Mat point2); // 3d point
	bool isPointOnVectorLine(Point2f point, Point2f startp, Point2f endp);
	vector<int> generateIntAscentArray(int size);
	int findBottomLeftPointIdx(vector<Point2f> fourPoints);
};

