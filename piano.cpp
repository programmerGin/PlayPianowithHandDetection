#include <opencv2/opencv.hpp>
#include <Windows.h>
#include <MMSystem.h>
#pragma comment(lib,"Winmm.lib")
using namespace cv;
using namespace std;

VideoCapture capture;
int erosion_value = 0;
int const max_erosion = 2;
int erosion_size = 0;
int const ersion_max_size = 21;
int dilation_value = 0;
int dilation_size = 0;
int erosion_type = 0;
int cnt = 0;
int loc, tmp = 0;
int touch_cnt = 0;



Mat getHandMask(const Mat& image, int minCr = 133, int maxCr = 173, int minCb = 77, int maxCb = 127) {
	Mat result;
	cvtColor(image, result, COLOR_BGR2YCrCb);
	vector<Mat> planes;
	split(result, planes);
	Mat mask = (minCr < planes[1]) & (planes[1] < maxCr) & (minCb < planes[2]) & (planes[2] < maxCb);
	erode(mask, mask, Mat(3, 3, CV_8U, Scalar(1)), Point(-1, -1), 2);
	return mask;
}



float innerAngle(float px1, float py1, float px2, float py2, float cx1, float cy1)
{

	float dist1 = std::sqrt((px1 - cx1) * (px1 - cx1) + (py1 - cy1) * (py1 - cy1));
	float dist2 = std::sqrt((px2 - cx1) * (px2 - cx1) + (py2 - cy1) * (py2 - cy1));

	float Ax, Ay;
	float Bx, By;
	float Cx, Cy;

	Cx = cx1;
	Cy = cy1;
	if (dist1 < dist2)
	{
		Bx = px1;
		By = py1;
		Ax = px2;
		Ay = py2;


	}
	else {
		Bx = px2;
		By = py2;
		Ax = px1;
		Ay = py1;
	}


	float Q1 = Cx - Ax;
	float Q2 = Cy - Ay;
	float P1 = Bx - Ax;
	float P2 = By - Ay;


	float A = std::acos((P1 * Q1 + P2 * Q2) / (std::sqrt(P1 * P1 + P2 * P2) * std::sqrt(Q1 * Q1 + Q2 * Q2)));

	A = A * 180 / CV_PI;

	return A;
}



void play_Sound(int fingercnt) {
	switch (fingercnt) {
	case 1:
		sndPlaySoundA("sound1.wav", SND_ASYNC | SND_NODEFAULT);
		Sleep(50);
		break;
	case 2:
		sndPlaySoundA("sound2.wav", SND_ASYNC | SND_NODEFAULT);
		Sleep(50);
		break;
	case 3:
		sndPlaySoundA("sound3.wav", SND_ASYNC | SND_NODEFAULT);
		Sleep(50);
		break;
	case 4:
		sndPlaySoundA("sound4.wav", SND_ASYNC | SND_NODEFAULT);
		Sleep(50);
		break;
	case 5:
		sndPlaySoundA("sound5.wav", SND_ASYNC | SND_NODEFAULT);
		Sleep(50);
		break;
	}
}

void finger_location(Point p) {
	cout << p.x << endl;
	if (p.y < 230) {
		if (p.x > 0 && p.x < 80) {
			cout << "도" << endl;
			loc = 1;
		}
		else if (p.x > 230 && p.x < 300) {
			cout << "레" << endl;
			loc = 2;
		}
		else if (p.x > 445 && p.x < 515) {
			cout << "미" << endl;
			loc = 3;
		}
		else if (p.x > 660 && p.x < 730) {
			cout << "파" << endl;
			loc = 4;
		}
		else if (p.x > 880 && p.x < 960) {
			cout << "솔" << endl;
			loc = 5;
		}
		if (tmp != loc) {
			play_Sound(loc);
			touch_cnt = 0;
		}
		else {
			touch_cnt++;
			if (touch_cnt == 7) {
				play_Sound(loc);
				touch_cnt = 0;
			}
		}
		tmp = loc;

	}
}


int main() {
	Mat frame;
	Mat element;
	Mat handImg;
	Mat mask, dst;
	Mat key, key_th, k_mask[5];
	Mat foreground, background;
	vector<std::vector< Point>> contours;
	vector< Vec4i> hierarchy;
	double radius = 5;
	capture.open(0);

	capture.set(CAP_PROP_FRAME_WIDTH, 960);
	capture.set(CAP_PROP_FRAME_HEIGHT, 540);
	capture.set(CAP_PROP_AUTOFOCUS, 0);

	int zoom = capture.get(CAP_PROP_ZOOM);
	int focus = capture.get(CAP_PROP_FOCUS);

	string title = "핑거 피아노";
	key = imread("piano.jpg");
	threshold(key, key_th, 70, 255, THRESH_BINARY);
	split(key_th, k_mask);

	bitwise_or(k_mask[0], k_mask[1], k_mask[3]);
	bitwise_or(k_mask[2], k_mask[3], k_mask[3]);
	bitwise_not(k_mask[3], k_mask[4]);

	namedWindow(title);
	//createTrackbar("ele_erosion", title, &erosion_value, max_erosion);
	//createTrackbar("erosion_size", title, &erosion_size, ersion_max_size);
	//createTrackbar("ele_dilation", title, &dilation_value, max_erosion);
	//createTrackbar("dilation_size", title, &dilation_size, ersion_max_size);



	while (true) {

		capture >> frame;

		//frame = imread("D:/kimhand.png");
		flip(frame, frame, 1);
		//imshow("original_image", frame);



		if (erosion_value == 0) erosion_type = MORPH_RECT;
		else if (erosion_value == 1) erosion_type = MORPH_CROSS;
		else if (erosion_value == 2) erosion_type = MORPH_ELLIPSE;

		element = getStructuringElement(erosion_type, Size(2 * erosion_size + 1, 2 * erosion_size + 1), Point(erosion_size, erosion_size));
		//cvtColor(frame, handImg, CV_BGR2YCrCb);
		//inRange(handImg,  Scalar(0, 133, 77),  Scalar(255, 173, 127), handImg);
		//cvtColor(handImg, handImg, CV_YCrCb2BGR);
		//cvtColor(handImg, handImg, CV_BGR2GRAY);
		//threshold(handImg, handImg, 130, 255, THRESH_BINARY| THRESH_OTSU);
		handImg = getHandMask(frame).clone();
		//imshow("이진화", handImg);
		erode(handImg, handImg, element);
		dilate(handImg, handImg, element);
		mask = handImg.clone();
		findContours(mask, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE, Point(0, 0));
		int largestContour = 0;
		for (int i = 0; i < contours.size(); i++) {
			if (contourArea(contours[i]) > contourArea(contours[largestContour])) {
				largestContour = i;
			}
		}
		drawContours(frame, contours, largestContour, Scalar(0, 255, 255), 1, 8, vector < Vec4i>(), 0, Point());
		if (!contours.empty()) {
			vector<vector<Point>>hull(1);
			convexHull(Mat(contours[largestContour]), hull[0], false);
			drawContours(frame, hull, 0, Scalar(0, 255, 0), 1, 8, vector<Vec4i>(), 0, Point());

			if (hull[0].size() > 2) {

				Rect boundingBox = boundingRect(hull[0]);
				rectangle(frame, boundingBox, Scalar(255), 2);
				Point center = Point(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);

				vector<Point> validPoints;

				vector<int> hullIndex;
				convexHull(cv::Mat(contours[largestContour]), hullIndex);
				vector<Vec4i> convexityDefects;

				cv::convexityDefects(Mat(contours[largestContour]), hullIndex, convexityDefects);

				for (int i = 0; i < convexityDefects.size(); i++) {
					Point p1 = contours[largestContour][convexityDefects[i][0]];//start
					Point p2 = contours[largestContour][convexityDefects[i][1]];//end
					Point p3 = contours[largestContour][convexityDefects[i][2]];//far


					double angle = atan2(center.y - p1.y, center.x - p1.x) * 180 / CV_PI;
					double inA = innerAngle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
					double length = sqrt(pow(p1.x - p3.x, 2) + pow(p1.y - p3.y, 2));



					if (angle > -30 && angle < 160 && abs(inA) > 10 && abs(inA) < 130 && length > 0.2 * boundingBox.height)
					{
						validPoints.push_back(p1);
					}
				}

				Point finger;

				if (validPoints.size() != 0) {

					for (int i = 0; i < 1; i++)
					{

						circle(frame, validPoints[i], 9, Scalar(255, 0, 0), 2);
						finger = validPoints[i];
					}
				}
				if (cnt != validPoints.size()) {
					//play_Sound(validPoints.size());
					finger_location(finger);

				}
				else if (cnt == 0) {

				}
				cnt = validPoints.size();
				Sleep(10);


			}
		}




		Point center1 = frame.size() / 2;
		Point center2 = key.size() / 2;
		Point start = center1 - center2;
		Rect roi(start, key.size());

		bitwise_and(key, key, foreground, k_mask[3]);
		bitwise_and(frame(roi), frame(roi), background, k_mask[4]);

		add(background, foreground, dst);

		imshow(title, dst);

		if (waitKey(10) == 27) break;
	}
	destroyAllWindows();
	return 0;
}