#include <opencv2/opencv.hpp>
#include <Windows.h>
#include <MMSystem.h>
#pragma comment(lib,"Winmm.lib")
using namespace cv;
using namespace std;

int loc, tmp, touch_cnt = 0;
VideoCapture capture(0);
vector<Point> cons[100];

int minCr = 133;
int maxCr = 173;
int minCb = 77;
int maxCb = 127;
int pianoCnt = 0;

Mat getHandMask(const Mat& image) { 
    Mat result;
    cvtColor(image, result, COLOR_BGR2YCrCb); 
    vector<Mat> planes;
    split(result, planes); 
    Mat mask = (minCr < planes[1]) & (planes[1] < maxCr) & (minCb < planes[2]) & (planes[2] < maxCb);
    blur(mask, mask, Size(3, 3));
    //medianBlur(mask, mask, 5);
    morphologyEx(mask, mask, MORPH_CLOSE, Mat(), Point(-1, -1), 1);
    //erode(mask, mask, Mat(5, 5, CV_8U, Scalar(1)), Point(-1, -1), 2);
    return mask;
}
void play_Sound(int fingercnt) {
    switch (fingercnt) {
    case 1:
        sndPlaySoundA("./audio/do.wav", SND_ASYNC | SND_NODEFAULT | SND_NOSTOP);
        break;
    case 2:
        sndPlaySoundA("./audio/rae.wav", SND_ASYNC | SND_NODEFAULT | SND_NOSTOP);
        break;
    case 3:
        sndPlaySoundA("./audio/mi.wav", SND_ASYNC | SND_NODEFAULT | SND_NOSTOP);
        break;
    case 4:
        sndPlaySoundA("./audio/fa.wav", SND_ASYNC | SND_NODEFAULT | SND_NOSTOP);
        break;
    case 5:
        sndPlaySoundA("./audio/sol.wav", SND_ASYNC | SND_NODEFAULT | SND_NOSTOP);
        break;
    case 6:
        sndPlaySoundA("./audio/la.wav", SND_ASYNC | SND_NODEFAULT | SND_NOSTOP);
        break;
    case 7:
        sndPlaySoundA("./audio/ti.wav", SND_ASYNC | SND_NODEFAULT | SND_NOSTOP);
        break;
    case 8:
        sndPlaySoundA("./audio/do_h.wav", SND_ASYNC | SND_NODEFAULT | SND_NOSTOP);
        break;
    }
}

int finger_location(Point p) {
    if (pointPolygonTest(cons[0], p, false) == 1) {
        cout << "도" << endl;
        loc = 1;
        play_Sound(loc);
    }
    else if (pointPolygonTest(cons[1], p, false) == 1) {
        cout << "레" << endl;
        loc = 2;
        play_Sound(loc);
    }
    else if (pointPolygonTest(cons[2], p, false) == 1) {
        cout << "미" << endl;
        loc = 3;
        play_Sound(loc);
    }
    else if (pointPolygonTest(cons[3], p, false) == 1) {
        cout << "파" << endl;
        loc = 4;
        play_Sound(loc);
    }
    else if (pointPolygonTest(cons[4], p, false) == 1) {
        cout << "솔" << endl;
        loc = 5;
        play_Sound(loc);
    }
    else if (pointPolygonTest(cons[5], p, false) == 1) {
        cout << "라" << endl;
        loc = 6;
        play_Sound(loc);
    }
    else if (pointPolygonTest(cons[6], p, false) == 1) {
        cout << "시" << endl;
        loc = 7;
        play_Sound(loc);
    }
    else if (pointPolygonTest(cons[7], p, false) == 1) {
        cout << "도" << endl;
        loc = 8;
        play_Sound(loc);
    }
    Sleep(10);
    return loc;
}


float innerAngle(float px1, float py1, float px2, float py2, float cx1, float cy1)
{

    float dist1 = sqrt((px1 - cx1) * (px1 - cx1) + (py1 - cy1) * (py1 - cy1));
    float dist2 = sqrt((px2 - cx1) * (px2 - cx1) + (py2 - cy1) * (py2 - cy1));

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

    float A = acos((P1 * Q1 + P2 * Q2) / (sqrt(P1 * P1 + P2 * P2) * sqrt(Q1 * Q1 + Q2 * Q2)));

    A = A * 180 / CV_PI;

    return A;
}

void on_minCr_change(int pos, void* userdata)
{
    minCr = pos;

    imshow("Tool", NULL);

}

void on_maxCr_change(int pos, void* userdata)
{
    maxCr = pos;

    imshow("Tool", NULL);

}

void on_minCb_change(int pos, void* userdata)
{
    minCb = pos;

    imshow("Tool", NULL);

}

void on_maxCb_change(int pos, void* userdata)
{
    maxCb = pos;

    imshow("Tool", NULL);

}

void piano_shot(int pos, void* userdata)
{
    if (pos == 1)
    {
        pianoCnt = 1;
    }
}

int main() {
    Mat frame;
    Mat handImg;
    Mat mask;
    Mat lineMask;

    vector<vector< Point>> contours;
    vector<vector< Point>> con;
    vector< Vec4i> hierarchy;

    string title = "가상 악기 피아노 프로젝트";

    namedWindow("Tool", WINDOW_GUI_EXPANDED);

    createTrackbar("minCr", "Tool", 0, 255, on_minCr_change);
    createTrackbar("maxCr", "Tool", 0, 255, on_maxCr_change);
    createTrackbar("micCb", "Tool", 0, 255, on_minCb_change);
    createTrackbar("maxCb", "Tool", 0, 255, on_maxCb_change);

    namedWindow(title);
    createTrackbar("findPiano", title, 0, 1, piano_shot);

    while (true) {
        capture >> frame;
        flip(frame, frame, 1);
        if (pianoCnt == 1)
        {
            cvtColor(frame, lineMask, COLOR_BGR2GRAY);
            adaptiveThreshold(lineMask, lineMask, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 21, 5);

            //imshow("thres", lineMask);
            findContours(lineMask, con, RETR_TREE, CHAIN_APPROX_TC89_L1);
            pianoCnt++;
        }


        handImg = getHandMask(frame).clone();

        //imshow("이진화", handImg);
        mask = handImg.clone();
        findContours(mask, contours, hierarchy, RETR_LIST, CHAIN_APPROX_TC89_L1);
        int largestContour = 0;

        for (int k = 0; k < contours.size(); k++) {
            if (contourArea(contours[k]) > 6000) {
                largestContour = k;
                drawContours(frame, contours, largestContour, Scalar(255, 210, 90), 2, 8, vector < Vec4i>(), 0, Point());
                if (!contours.empty()) {
                    vector<vector<Point>>hull(1);
                    convexHull(Mat(contours[largestContour]), hull[0], false);
                    drawContours(frame, hull, 0, Scalar(0, 255, 0));

                    if (hull[0].size() > 2) {

                        Rect boundingBox = boundingRect(hull[0]);
                        //rectangle(frame, boundingBox, Scalar(255), 2);
                        Point center = Point(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);

                        vector<Point> validPoints;

                        vector<int> hullIndex;
                        convexHull(Mat(contours[largestContour]), hullIndex);
                        vector<Vec4i> convexityDefect;

                        convexityDefects(Mat(contours[largestContour]), hullIndex, convexityDefect);

                        for (int i = 0; i < convexityDefect.size(); i++) {
                            Point p1 = contours[largestContour][convexityDefect[i][0]];//start
                            Point p2 = contours[largestContour][convexityDefect[i][1]];//end
                            Point p3 = contours[largestContour][convexityDefect[i][2]];//far

                            circle(frame, p1, 3, Scalar(0, 0, 255), 1);
                            circle(frame, p2, 3, Scalar(0, 255, 0), 1);
                            circle(frame, p3, 3, Scalar(225, 0, 0), 1);

                            line(frame, p1, p3, Scalar(0, 0, 0), 1);
                            line(frame, p3, p2, Scalar(0, 0, 0), 1);

                            double angle = atan2(center.y - p1.y, center.x - p1.x) * 180 / CV_PI;
                            double inA = innerAngle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
                            double length = sqrt(pow(p1.x - p3.x, 2) + pow(p1.y - p3.y, 2));


                            if (angle > -360 && angle < 30 && abs(inA) > 5 && abs(inA) < 120 && length > 0.2 * boundingBox.height) //-40
                            {
                                validPoints.push_back(p1);
                            }
                        }

                        if (validPoints.size() != 0) {

                            for (int i = 0; i < validPoints.size(); i++)
                            {
                                Point finger = validPoints[i];
                                circle(frame, validPoints[i], 9, Scalar(255, 0, 0), 2);

                                if (!cons[0].empty())
                                {
                                    //finger_location(finger);
                                    int a = finger_location(finger);
                                    //fillPoly(frame, cons[a-1], Scalar(0, 255, 0));
                                }

                            }
                        }//cout << "손가락 갯수" << validPoints.size() << endl;

                    }
                }


            }
        }

        vector<Point> approxCurve;
        int consNum = 0;
        //건반 첫 출력
        for (int i = 0; i < con.size(); i++)
        {
            if (10000 < contourArea(con[i]))
            {
                if (60000 > contourArea(con[i]))
                {
                    drawContours(frame, con, i, Scalar(0, 0, 0), 2);

                    cons[consNum] = con[i];
                    consNum++;
                }
            }
        }
        imshow(title, frame);
        imshow("Tool", NULL);
        if (waitKey(10) == 27) break;
    }
    destroyAllWindows();
    return 0;
}