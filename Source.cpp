#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <Windows.h>
using namespace cv;
using namespace std;


float innerAngle(float px1, float py1, float px2, float py2, float cx1, float cy1);

int main(void)
{
    //배경 제거 변수 생성
    Ptr<BackgroundSubtractor> pBackSub;
    pBackSub = createBackgroundSubtractorMOG2();

    VideoCapture capture(0);
    if (!capture.isOpened()) {
        //비디오 에러 문구
        cerr << "Unable to open" << endl;
        return 1;
    }

    //흑백 화면
    Mat hand;
    //컬러 화면, 이진화 마스크
    Mat frame, fgMask;
    while (true) {
        capture >> frame;
        if (frame.empty())
            break;
        //배경 업데이트
        pBackSub->apply(frame, fgMask);

        //컬러 화면에 피아노 부위
        Mat pianoRoi = fgMask(Rect(100, 100, 100, 100));

        //좌우반전
        flip(frame, frame, 1);
        flip(fgMask, fgMask, 1);

        //피아노 출력
        rectangle(frame, Rect(100, 100, 100, 100), Scalar(255, 0, 0));

        cvtColor(frame, hand, COLOR_BGR2GRAY);
        GaussianBlur(fgMask, fgMask, Size(7, 7), 0);

        //윤곽 변수
        vector<vector<Point>> contours;
        findContours(fgMask, contours, RETR_EXTERNAL, CHAIN_APPROX_TC89_L1);

        //윤곽 모양
        vector<vector<Point> >contours_poly(contours.size());
        //사각형 윤곽
        vector<Rect> boundRect(contours.size());

        size_t largestContour = 0;
        for (int i = 0; i < contours.size(); i++)
        {
            if (contourArea(contours[i]) > contourArea(contours[largestContour]))
                largestContour = i;
            if (contourArea(contours[i]) > 4000) {
                approxPolyDP(contours[i], contours_poly[i], arcLength(contours[i], true) * 0.02, true);
                boundRect[i] = boundingRect(contours_poly[i]);
                rectangle(hand, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 0, 255), 2, 8, 0);
                drawContours(hand, contours, largestContour, 255, 2, 8, vector<Vec4i>());

                if (!contours.empty())
                {
                    //볼록 외피(convexHull)
                    vector<vector<Point> > hull(1);
                    convexHull(Mat(contours[largestContour]), hull[0], false);
                    drawContours(frame, hull, 0, Scalar(0, 255, 0), 3);
                    if (hull[0].size() > 1)
                    {
                        vector<int> hullIndexes;
                        convexHull(Mat(contours[largestContour]), hullIndexes, true);
                        vector<Vec4i> convexityDefect;
                        convexityDefects(Mat(contours[largestContour]), hullIndexes, convexityDefect);
                        Rect boundingBox = boundingRect(hull[0]);
                        rectangle(frame, boundingBox, Scalar(255, 0, 0));
                        Point center = Point(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
                        vector<Point> validPoints;
                        for (size_t i = 0; i < convexityDefect.size(); i++)
                        {
                            Point p1 = contours[largestContour][convexityDefect[i][0]];
                            Point p2 = contours[largestContour][convexityDefect[i][1]];
                            Point p3 = contours[largestContour][convexityDefect[i][2]];
                            double angle = atan2(center.y - p1.y, center.x - p1.x) * 180 / CV_PI;
                            double inAngle = innerAngle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
                            double length = sqrt(pow(p1.x - p3.x, 2) + pow(p1.y - p3.y, 2));
                            if (angle > -30 && angle < 160 && abs(inAngle) > 20 && abs(inAngle) < 120 && length > 0.1 * boundingBox.height)
                            {
                                validPoints.push_back(p1);
                            }
                        }
                        for (size_t i = 0; i < validPoints.size(); i++)
                        {
                            circle(frame, validPoints[i], 9, Scalar(0, 255, 0), 2);
                            if (100 < validPoints[i].x < 200 && 100 < validPoints[i].y < 200)
                            {
                               // PlaySound(TEXT("./audio/do.wav"), NULL, SND_FILENAME | SND_ASYNC);
                            }
                        }
                    }
                }

            }
        }

        imshow("Frame", frame);
        imshow("FG Mask", hand);
        //esc키로 종료
        int keyboard = waitKey(30);
        if (keyboard == 'q' || keyboard == 27)
            break;
    }
    return 0;
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