#include "Tracker.h"

void Tracker::processFrame(Mat &img_input, Mat &out) {
	img_input.copyTo(img_out);

	// 1. Binarize the image
	binarizeImg(img_out, img_binary);
	imshow("binarized", img_binary);
	waitKey(1);

	// 2. Find contours & points
	
	findContours(img_binary, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
	
	if (contours.size() == 0) {
		out = img_out;
		endProcessFrame();
		return;
	}

	// 3. Process each contours:각 contour마다 6 points 저장 & corner point를 찾는다.
	for (size_t i = 0; i < contours.size(); i++) {
		if (contourArea(contours[i]) > minCntSize)
			processContours(contours[i]);
	}

	 //Draw
	//for (int i = 0; i < cornerPoints.size(); i++) {
	//	circle(img_out, cornerPoints[i], 3, Scalar(0, 0, 255));
	//}

	if (markerPoints.size() == 0) {
		out = img_out;
		endProcessFrame();
		return;
	}
	
	// 4. Grouping
	grouping(); // CCW

	deleteUncompletedGroup();
	if (groupedPoints.size() == 0) {
		out = img_out;
		endProcessFrame();
		return;
	}

	sortEachGroup(); //find left-below point & sort by CCW
	//sortEachCornerGroup();

	//circle(img_out, groupedPoints[0][0], 3, Scalar(0, 0, 255));
	//circle(img_out, groupedPoints[0][1], 3, Scalar(0, 255, 0));
	//circle(img_out, groupedPoints[0][2], 3, Scalar(255, 0, 0));
	//circle(img_out, groupedPoints[0][3], 3, Scalar(0, 0, 0));

	//draw line
	//for (int i=0; i < groupedPoints.size(); i++) {
	//	for (int j=0; j <groupedPoints[i].size(); j++)
	//		line(img_out, groupedPoints[i][j], groupedPoints[i][(j+1) % groupedPoints[i].size()], Scalar(0, 255, 0));
	//}

	// 4.5 Get image square points of each marker
	for (int i = 0; i < groupedMarkerIndex.size(); i++) {
		for (int j = 0; j < groupedMarkerIndex[i].size(); j++) {
			 
			int index = groupedMarkerIndex[i][j];
			vector<Point2f> fourPoints = findImgSquarePointsOfMarker(index);

			pair<int, vector<Point2f>> p(index, fourPoints);
			markerImgSquarePoints.insert(p);
		}
	}

	// 5. Get Camera Pose for each marker
	double m[] = { fx, 0, cx, 0, fy, cy, 0, 0, 1 };
	camIntrinsic = Mat(3, 3, CV_64FC1, m);

	double d[] = { k1, k2, p1, p2 };
	distCoeffs = Mat(4, 1, CV_64FC1, d);
	
	map<int, vector<Point2f>>::iterator iter;
	for (iter = markerImgSquarePoints.begin(); iter != markerImgSquarePoints.end(); ++iter)
	{	
		int index = (*iter).first;
		vector<Point2f> squarePoints = (*iter).second;
		squarePoints.push_back(getCenterPoint(index));

		//solve
		Mat rvec, tvec;
		solvePnP(markerSquarePoints3d, squarePoints, camIntrinsic, distCoeffs, rvec, tvec, 0, CV_ITERATIVE);
	
		circle(img_out, squarePoints[0], 3, Scalar(0, 0, 255));
		circle(img_out, squarePoints[1], 3, Scalar(0, 255, 0));
		circle(img_out, squarePoints[2], 3, Scalar(255, 0, 0));
		circle(img_out, squarePoints[3], 3, Scalar(0, 0, 0));
				
		Mat rotationMat;
		Rodrigues(rvec, rotationMat);

		//save
		pair<int, Mat> rp(index, rvec);
		pair<int, Mat> tp(index, tvec);
		cameraRvecs.insert(rp);
		cameraTvecs.insert(tp);
	}

	// 5.5. Get length & width of group & print
	for (int i = 0; i < groupedMarkerIndex.size(); i++) {

		int index1 = groupedMarkerIndex[i][0];
		int index2 = groupedMarkerIndex[i][1];
		int index3 = groupedMarkerIndex[i][2];
		int index4 = groupedMarkerIndex[i][3];

		Mat pos1 = cameraTvecs[index1];
		Mat pos2 = cameraTvecs[index2];
		Mat pos3 = cameraTvecs[index3];
		Mat pos4 = cameraTvecs[index4];

		double dist1 = euclidean3D(pos1, pos2);
		double dist2 = euclidean3D(pos2, pos3);
		double dist3 = euclidean3D(pos3, pos4);
		double dist4 = euclidean3D(pos4, pos1);

		double width = (dist1 + dist3) / 2;
		double height = (dist2 + dist4) / 2;

		widthHeightOfGroup.resize(i + 1);
		widthHeightOfGroup[i].push_back(width);
		widthHeightOfGroup[i].push_back(height);

		//cout << "dist1:" << dist1 << endl;
		//cout << "dist2:" << dist2 << endl;
		//cout << "dist3:" << dist3 << endl;
		//cout << "dist4:" << dist4 << endl;
		
		//소수점 2째 자리까지 나타내기
		stringstream ss1;
		ss1 << fixed << setprecision(2) << width;
		String text1 = "width: " + ss1.str();

		stringstream ss2;
		ss2 << fixed << setprecision(2) << height;
		String text2 = "height: " + ss2.str();

		Point2f point1 = (groupedPoints[i][0] + groupedPoints[i][1] + groupedPoints[i][2] + groupedPoints[i][3]) / 4;
		Point2f point2 = point1;
		point1.x = point1.x - 60;		
		point2.x = point2.x - 60;
		point2.y = point2.y + 30;

		putText(img_out, text1, point1, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,0));
		putText(img_out, text2, point2, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0));

		//cout << "group index: " << i << endl;
		//cout << "width: " << width << endl;
		//cout << "height: " << height << endl;
	}

	// Get camera pose for each group
	//getCameraPosForGroups();

	// 6. Draw cube for each group
	// draw by each marker's camera poisition
	drawByMarkerPos();

	// draw by group's camera position
	//drawByGroupPos();


	// 7. End process frame
	out = img_out;
	endProcessFrame();
	return;
}

void Tracker::endProcessFrame() {
	contours.clear();
	markerPoints.clear();
	cornerPoints.clear();
	cornerIdx.clear();
	groupedPoints.clear();
	groupedMarkerIndex.clear();
	widthHeightOfGroup.clear();
	markerImgSquarePoints.clear();
	cameraRvecs.clear();
	cameraTvecs.clear();
	cameraRvecsForGroup.clear();
	cameraTvecsForGroup.clear();
}

void Tracker::binarizeImg(const Mat &img_input, Mat & out) {
	
	//convert RGB to LAB
	cvtColor(img_input, img_hsv, COLOR_BGR2Lab);

	//binarize LAB image
	inRange(img_hsv, Scalar(10, 10, 170), Scalar(255,255,255), img_binary);

	//morphological opening (removal of small protrusions, thin connections) -잡티제거
	erode(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing (removal of holes) -hole 제거
	dilate(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
}

void Tracker::processContours(const vector<Point> contour) {

	//contours에서 점 추출
	vector<Point2f> approx;
	approxPolyDP(Mat(contour), approx, arcLength(Mat(contour), true)*EPSILON, true);
	
	if (approx.size() == NUM_MARKER_POINTS) 
	{	
		//1. find 2 max edges
		int firstEdgePoints[2];
		int secondEdgePoints[2];
		findTwoMaxEdges(approx, firstEdgePoints, secondEdgePoints);

		// 2. Find the corner point
		int cornerPointIdx = findOverlappedPoint(firstEdgePoints, secondEdgePoints);
		
		// 3. save
		if(cornerPointIdx> -1 && cornerPointIdx < 6){			
			markerPoints.push_back(approx);
			cornerPoints.push_back(approx[cornerPointIdx]);
			cornerIdx.push_back(cornerPointIdx);
		}
	}
}

void Tracker::findTwoMaxEdges(vector<Point2f> approx, int firstEdgePoints[], int secondEdgePoints[])
{	
	int size = approx.size();

	float firstD = -1.0f, secondD = -1.0f;
	float dist;

	// find the max dist points
	for (int k = 0; k < size; k++) {
		dist = euclideanD(approx[k], approx[(k + 1) % size]);		
		if (dist > firstD) {
			firstD = dist;
			firstEdgePoints[0] = k;
			firstEdgePoints[1] = (k + 1) % size;
		}
	}

	// find the second max dist points. It should include one of the firstD's points.
	int temp = firstEdgePoints[0];	
	float dist1 = euclideanD(approx[temp], approx[(temp + 5) % size]);
	float dist2 = euclideanD(approx[(temp + 1) % size], approx[(temp + 2) % size]);

	if (dist1 > dist2) {
		secondEdgePoints[0] = temp;
		secondEdgePoints[1] = (temp + 5) % size;
	}
	else {
		secondEdgePoints[0] = (temp + 1) % size;
		secondEdgePoints[1] = (temp + 2) % size;
	}

	//line(img_out, approx[firstEdgePoints[0]], approx[firstEdgePoints[1]], Scalar(0, 255, 0));
	//line(img_out, approx[secondEdgePoints[0]], approx[secondEdgePoints[1]], Scalar(0, 0, 0));
}

int Tracker::findOverlappedPoint(int array1[], int array2[]) {
	int cornerPoint = -2;
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++)
			if (array1[i] == array2[j])
				cornerPoint = array1[i];
	}
	if(cornerPoint>-1)
		return cornerPoint;

}

void Tracker::grouping() {
	vector<int> leftMarkers = generateIntAscentArray(markerPoints.size());

	int k = 0;
	while (!leftMarkers.empty()) {

		// left markers의 첫 번째를 기준으로 시작 
		int curMarker = leftMarkers[0];

		leftMarkers.erase(leftMarkers.begin());
		cornerPoints[curMarker];

		groupedPoints.resize(k + 1);
		groupedMarkerIndex.resize(k + 1);

		groupedPoints[k].push_back(getCenterPoint(curMarker));
		groupedMarkerIndex[k].push_back(curMarker);


		// 기준을 시작으로 하나의 group 찾기
		while (groupedPoints[k].size() < 4) {

			// Find the next marker index - the point on the line connecting current marker's corner & the neighbor of the corner. 
			// The 4 markers are found by CCW.
			int nextMarker = -1;
			float minD = FLT_MAX;

			for (int i = 0; i < leftMarkers.size(); i++)
			{

				if (isPointOnVectorLine(cornerPoints[leftMarkers[i]], cornerPoints[curMarker], getConerRightPoint(curMarker)) // on the line
					&& euclideanD(cornerPoints[leftMarkers[i]], cornerPoints[curMarker]) < minD // check the distance
					&& leftMarkers[i] != curMarker)
				{
					nextMarker = leftMarkers[i];
					minD = euclideanD(cornerPoints[leftMarkers[i]], cornerPoints[curMarker]);
				}
			}

			if (nextMarker < 0) { // no next marker in the image. Proceed the next group.
				break;
			}
			else {
				//remove from leftMarkers
				std::vector<int>::iterator position = std::find(leftMarkers.begin(), leftMarkers.end(), nextMarker);
				if (position != leftMarkers.end()) // == myVector.end() means the element was not found
					leftMarkers.erase(position);

				groupedPoints[k].push_back(getCenterPoint(nextMarker));
				groupedMarkerIndex[k].push_back(nextMarker);
				curMarker = nextMarker;
			}
		}

		//다시 새로운 group을 찾는다.
		k++;
	}
}

void Tracker::deleteUncompletedGroup() {
	vector<vector<Point2f>>::iterator it;
	for (it = groupedPoints.begin(); it != groupedPoints.end();) {
		if (it->size() != 4)
			it = groupedPoints.erase(it);
		else
			it++;
	}

	vector<vector<int>>::iterator it2;
	for (it2 = groupedMarkerIndex.begin(); it2 != groupedMarkerIndex.end();) {
		if (it2->size() != 4)
			it2 = groupedMarkerIndex.erase(it2);
		else
			it2++;
	}

}

void Tracker::sortEachGroup() {
	for (int i = 0; i < groupedPoints.size(); i++) {
		
		//copy
		vector<Point2f> temp;
		for (int k = 0; k < groupedPoints[i].size(); k++) {
			temp.push_back(groupedPoints[i][k]);
		}
		
		// Find the most left point(if same, below point) index
		int idx = findBottomLeftPointIdx(groupedPoints[i]);
		groupedPoints[i][0] = temp[idx];
		groupedPoints[i][1] = temp[(idx+1)%4];
		groupedPoints[i][2] = temp[(idx+2)%4];
		groupedPoints[i][3] = temp[(idx+3)%4];

		//// Make the most left point(if same, below point) be at [0]
		//sort(groupedPoints[i].begin(), groupedPoints[i].end(), markerSquareSorter);

		//// Make the order be CCW
		//swap(groupedPoints[i][2], groupedPoints[i][3]);
		//Point2f v1 = groupedPoints[i][1] - groupedPoints[i][0];
		//Point2f v2 = groupedPoints[i][2] - groupedPoints[i][0];

		//double o = (v1.x * v2.y) - (v1.y * v2.x);
		//if (o > 0.0)
		//	swap(groupedPoints[i][1], groupedPoints[i][3]);
	}
}

//void Tracker::sortEachCornerGroup() {
//	for (int i = 0; i < groupedCornerPoints.size(); i++) {
//		//copy
//		vector<Point2f> temp;
//		for (int k = 0; k < groupedCornerPoints[i].size(); k++) {
//			temp.push_back(groupedCornerPoints[i][k]);
//		}
//
//		// Find the most left point(if same, below point) index
//		int idx = findBottomLeftPointIdx(groupedCornerPoints[i]);
//		groupedCornerPoints[i][0] = temp[idx];
//		groupedCornerPoints[i][1] = temp[(idx + 1) % 4];
//		groupedCornerPoints[i][2] = temp[(idx + 2) % 4];
//		groupedCornerPoints[i][3] = temp[(idx + 3) % 4];
//	}
//}

Point2f Tracker::getConerRightPoint(int markerIdx) {

	Point2f corner = markerPoints[markerIdx].at(cornerIdx[markerIdx]);
	Point2f neighbor1 = markerPoints[markerIdx].at((cornerIdx[markerIdx] + 1) % NUM_MARKER_POINTS);
	Point2f neighbor2 = markerPoints[markerIdx].at((cornerIdx[markerIdx] + 5) % NUM_MARKER_POINTS);
	
	Point2f v1 = neighbor1 - corner;
	Point2f v2 = neighbor2 - corner;

	// check the cross product
	double o = (v1.x * v2.y) - (v1.y * v2.x);
	Point2f returnval;
	if (o < 0)
		returnval = neighbor1;
	else
		returnval =  neighbor2;

	return returnval;
}

Point2f Tracker::getCenterPoint(int markerIdx) {
	return markerPoints[markerIdx].at((cornerIdx[markerIdx] + 3) % NUM_MARKER_POINTS);
}

vector<Point2f> Tracker::findImgSquarePointsOfMarker(int markerIdx) {
	
	// Get six marker points & index
	vector<Point2f> sixMarkerPoints = markerPoints[markerIdx];
	int cornerPoint = cornerIdx[markerIdx];

	// Get the intersection of two lines (l1: defined by (cornerPoint+1)%6 & (cornerPoint+2)%6 , 
	// l2: defined by (intersectPoint+4)%6 & (intersectPoint+5)%6 
	Point2f point1 = sixMarkerPoints[(cornerPoint + 1) % 6];
	Point2f point2 = sixMarkerPoints[(cornerPoint + 2) % 6];
	Point2f point4 = sixMarkerPoints[(cornerPoint + 4) % 6];
	Point2f point5 = sixMarkerPoints[(cornerPoint + 5) % 6];

	float a[2][2] = { { point2.y - point1.y, point1.x - point2.x },
	{ point5.y - point4.y, point4.x - point5.x } };

	float b[2][1] = { { (point2.y - point1.y)*point1.x + (point1.x - point2.x)*point1.y },
	{ (point5.y - point4.y)*point4.x + (point4.x - point5.x)*point4.y } };

	Mat A = Mat(2, 2, CV_32FC1, a);
	Mat B = Mat(2, 1, CV_32FC1, b);
	Mat temp = A.inv() * B;

	Point2f fourthPoint;
	fourthPoint.x = temp.at<float>(0, 0);
	fourthPoint.y = temp.at<float>(1, 0);

	vector<Point2f> returnval;

	//put
	returnval.push_back(sixMarkerPoints[cornerPoint]);
	returnval.push_back(sixMarkerPoints[(cornerPoint + 1) % 6]);
	returnval.push_back(fourthPoint);
	returnval.push_back(sixMarkerPoints[(cornerPoint + 5) % 6]);

	//Make CCW
	Point2f v1 = returnval[1] - returnval[0];
	Point2f v2 = returnval[2] - returnval[0];

	double o = (v1.x * v2.y) - (v1.y * v2.x);
	
	if (o > 0.0)
		swap(returnval[1], returnval[3]);

	return returnval;
}

void  Tracker::getCameraPosForGroups() {
	for (int i = 0; i < groupedPoints.size(); i++) {

		double m[] = { fx, 0, cx, 0, fy, cy, 0, 0, 1 };
		camIntrinsic = Mat(3, 3, CV_64FC1, m);

		double d[] = { k1, k2, p1, p2 };
		distCoeffs = Mat(4, 1, CV_64FC1, d);

		vector<Point3f> groupSquarePoints3d;
		float width = widthHeightOfGroup[i][0];
		float height = widthHeightOfGroup[i][1];
		groupSquarePoints3d.push_back(Point3f(-width / 2, height / 2, 0));
		groupSquarePoints3d.push_back(Point3f(width / 2, height / 2, 0));
		groupSquarePoints3d.push_back(Point3f(width / 2, -height / 2, 0));
		groupSquarePoints3d.push_back(Point3f(-width / 2, -height / 2, 0));

		Mat rvec, tvec;
		solvePnP(groupSquarePoints3d, groupedPoints[i], camIntrinsic, distCoeffs, rvec, tvec, 0, CV_ITERATIVE);

		cameraRvecsForGroup.push_back(rvec);
		cameraTvecsForGroup.push_back(tvec);
	}
}

void Tracker::drawByMarkerPos() {
	for (int i = 0; i < groupedMarkerIndex.size(); i++) {

		// For a group
		vector<Point2f> imgCubeBottomPoints;
		vector<Point2f> imgCubeTopPoints;

		// For each marker
		for (int j = 0; j < groupedMarkerIndex[i].size(); j++) {

			int index = groupedMarkerIndex[i][j];
			Mat rvec = cameraRvecs[index];
			Mat tvec = cameraTvecs[index];

			Mat rotationMat;
			Rodrigues(rvec, rotationMat);
			cv::Mat_<float> rotMat = rotationMat;
			Mat result = rotMat * Mat(Vec3f(0, 1, 0));
			bool flip = result.at<float>(1, 0) > 0.0f;

			vector<Point3f> bottomTopPoints3d;
			bottomTopPoints3d.push_back(Point3f(0, 0, 0));
			bottomTopPoints3d.push_back(Point3f(0, flip ? -60 : 60, 0));

			vector<Point2f> bottomTopPoints2d;

			projectPoints(bottomTopPoints3d, rvec, tvec, camIntrinsic, distCoeffs, bottomTopPoints2d);
			imgCubeBottomPoints.push_back(bottomTopPoints2d[0]);
			imgCubeTopPoints.push_back(bottomTopPoints2d[1]);
		}

		//Draw 
		//Bottom
		for (int j = 0; j < 4; j++) {
			line(img_out, imgCubeBottomPoints[j], imgCubeBottomPoints[(j + 1) % 4], Scalar(0, 0, 0), 3);
			// circle(img_out, imgCubeBottomPoints[j], 1, Scalar(0, 0, 0));
		}

		//Top
		for (int j = 0; j < 4; j++) {
			line(img_out, imgCubeTopPoints[j], imgCubeTopPoints[(j + 1) % 4], Scalar(0, 0, 255), 3);
			//circle(img_out, imgCubeTopPoints[j], 1, Scalar(0, 0, 255));
		}

		//connect
		for (int j = 0; j < 4; j++) {
			line(img_out, imgCubeBottomPoints[j], imgCubeTopPoints[j], Scalar(0, 255, 0), 3);
		}
	}
}

void Tracker::drawByGroupPos() {
	for (int i = 0; i < groupedPoints.size(); i++) {

		vector<Point3f> cubeTopPoints3d;
		float width = widthHeightOfGroup[i][0];
		float height = widthHeightOfGroup[i][1];
		cubeTopPoints3d.push_back(Point3f(-width / 2, height / 2, 45));
		cubeTopPoints3d.push_back(Point3f(width / 2, height / 2, 45));
		cubeTopPoints3d.push_back(Point3f(width / 2, -height / 2, 45));
		cubeTopPoints3d.push_back(Point3f(-width / 2, -height / 2, 45));

		// Get 2D points
		vector<Point2f> imgBottomPoints = groupedPoints[i];

		vector<Point2f> imgTopPoints;
		double m[] = { fx, 0, cx, 0, fy, cy, 0, 0, 1 };
		camIntrinsic = Mat(3, 3, CV_64FC1, m);
		double d[] = { k1, k2, p1, p2 };
		distCoeffs = Mat(4, 1, CV_64FC1, d);
		projectPoints(cubeTopPoints3d, cameraRvecsForGroup[i], cameraTvecsForGroup[i], camIntrinsic, distCoeffs, imgTopPoints);

		//Draw 
		//Bottom
		for (int j = 0; j < 4; j++) {
			line(img_out, imgBottomPoints[j], imgBottomPoints[(j + 1) % 4], Scalar(0, 0, 0), 3);
			circle(img_out, imgBottomPoints[j], 3, Scalar(0, 0, 0));
		}

		//Top
		for (int j = 0; j < 4; j++) {
			line(img_out, imgTopPoints[j], imgTopPoints[(j + 1) % 4], Scalar(0, 0, 255), 3);
			circle(img_out, imgTopPoints[j], 3, Scalar(0, 0, 255));
		}

		//connect
		for (int j = 0; j < 4; j++) {
			line(img_out, imgBottomPoints[j], imgTopPoints[j], Scalar(0, 255, 0), 3);
		}
	}
}


// Util
float Tracker::euclideanD(Point2f point1, Point2f point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	float dist;

	dist = pow(x, 2) + pow(y, 2);
	dist = sqrt(dist);

	return dist;
}

double Tracker::euclidean3D(Mat point1, Mat point2)
{
	double x = point1.at<double>(0, 0) - point2.at<double>(0, 0);
	double y = point1.at<double>(1, 0) - point2.at<double>(1, 0);
	double z = point1.at<double>(2, 0) - point2.at<double>(2, 0);
	double dist;

	dist = pow(x, 2) + pow(y, 2) + pow(z, 2);
	dist = sqrt(dist);

	return dist;
}

// check if the [point] is on the vector connecting [startp] and [endp].
// err unit is pixel
bool Tracker::isPointOnVectorLine(Point2f point, Point2f startp, Point2f endp)
{	
	
	////line btw two points: y = ax + b
	//float diffy = endp.y - startp.y;
	//float diffx = endp.x - startp.x;
	//float a = diffy / diffx;
	//float b = startp.y - a*startp.x;

	//line btw two points: ax + by + c = 0
	float a = endp.y - startp.y;
	float b = startp.x - endp.x;
	float c = -(a*startp.x + b*startp.y);

	//distance between point and line
	float d = abs(a*point.x + b*point.y + c) / sqrt(pow(a, 2) + pow(b, 2));

	//cout << "startp: " << startp << endl;
	//cout << "endp: " << endp << endl;
	//cout << "point: " << point << endl;
	//cout << "err: " << err << endl;
	//cout << "measured err:" << d << endl;

	//cout << "a: " << a << endl;
	//cout << "b: " << b << endl;
	//cout << "c: " << c << endl;
	//cout << "d: " << d << endl;
	
	if (d < err){
		//check startp-endp, startp-point vector has same direction
		Point2f v1 = endp - startp;
		Point2f v2 = point - startp;
		//cout << "dot: " << v1.dot(v2) << endl;
		if (v1.dot(v2) >= 0)
			return true;
		else
			return false;
	}else
		return false;
}

vector<int> Tracker::generateIntAscentArray(int size) {
	vector<int> returnval;
	for (int i = 0; i < size; i++) {
		returnval.push_back(i);
	}
	return returnval;
}

int Tracker::findBottomLeftPointIdx(vector<Point2f> fourPoints) {
	float minx = FLT_MAX;
	int idx = -1;
	for (int i = 0; i < fourPoints.size(); i++) {
		if (fourPoints[i].x < minx) {
			minx = fourPoints[i].x;
			idx = i;
		}
		else if (fourPoints[i].x == minx) {
			if (fourPoints[idx].y < fourPoints[i].y)
				idx = i;
		}
	}
	return idx;
}


Tracker::Tracker()
{
	//todo: 이유는 모르겟지만 여기서 하면 initialize가 안 된다. 
	//여기로 나중에 옮기기.
	//double m[] = { fx, 0, cx, 0, fy, cy, 0, 0, 1 };
	//camIntrinsic = Mat(3, 3, CV_64FC1, m);
	//cout << camIntrinsic << endl;

	markerSquarePoints3d.push_back(Point3f(-30, 0, 30));
	markerSquarePoints3d.push_back(Point3f(30, 0, 30));
	markerSquarePoints3d.push_back(Point3f(30, 0, -30));
	markerSquarePoints3d.push_back(Point3f(-30, 0, -30));
	markerSquarePoints3d.push_back(Point3f(0, 0, 0));

	//cubeBottomPoints.push_back(Point3f(-30, 0, 30));
	//cubeBottomPoints.push_back(Point3f(30, 0, 30));
	//cubeBottomPoints.push_back(Point3f(30, 0, -30));
	//cubeBottomPoints.push_back(Point3f(-30, 0, -30));

	//cubeTopPoints.push_back(Point3f(-30, 60, 30));
	//cubeTopPoints.push_back(Point3f(30, 60, 30));
	//cubeTopPoints.push_back(Point3f(30, 60, -30));
	//cubeTopPoints.push_back(Point3f(-30, 60, -30));
}


Tracker::~Tracker()
{

}
