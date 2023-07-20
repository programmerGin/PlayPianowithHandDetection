#include <opencv2/opencv.hpp>
#include <Windows.h>
#include <MMSystem.h>
#pragma comment(lib,"Winmm.lib")
using namespace cv;
using namespace std;

int loc, tmp, touch_cnt = 0;
VideoCapture capture(0);
vector<Point> cons[100]; // Point 객체를 요소로 갖는 벡터(어레이) 선언. x,y 좌표 데이터가 한 셋트가 되어 저장됨.

int minCr = 135;
int maxCr = 173;
int minCb = 77;
int maxCb = 127;
int pianoCnt = 0;

Mat getHandMask(const Mat& image) { //파라미터에 손 검출 색상 영역을 넣어도 되지만 교수님께 배운 트랙바를 이용하기 위해 전역변수로 설정.
    Mat result;
    cvtColor(image, result, COLOR_BGR2YCrCb); // YCrCb로 영상의 색상을 변경 후 피부색을 동양인 표준 범위에 맞추어 손을 검출, BGR로 색상 변환. 색상 공간 변환(Convert Color): 본래의 색상 공간에서 다른 색상 공간으로 변환할 때 사용,  
    vector<Mat> planes; //mat타입을 저장할 벡터 planes 생성. vector 시퀀스 컨테이너/클래스 템플릿, 배열처럼 접근가능 + 새로운 데이터를 추가하거나 삭제 가능, 
    split(result, planes); // 3채널 result를 분리해서 각각의 다른 채널을 bgr 배열 요소에 담음. you separate the channels in a multichannels array into multiple single channel 
    Mat mask = (minCr < planes[1]) & (planes[1] < maxCr) & (minCb < planes[2]) & (planes[2] < maxCb);
    blur(mask, mask, Size(3, 3));
    //medianBlur(mask, mask, 5);
    morphologyEx(mask, mask, MORPH_CLOSE, Mat(), Point(-1, -1), 1); //침식과 팽창이 가지는 문제를 해결하기 위해 팽창 후 침식을 연이어 수행하는 확장 모폴로지 함수 (닫힘 연산)
    //MORPH_CLOSE 팽창 후 침식 수행, Kernel : 커널의 크기(검은구멍 매우는 정도 조절) , Anchor: mask의 중심점
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


   // 인자로 받은 점들의 안쪽 각을 검사하기위해 사용하는 함수이다
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

    vector<vector< Point>> contours; // x,y 좌표 데이터 셋트를 한 요소로 표현, 컨투어를 표현하는 점들의 집합
    vector<vector< Point>> con;
    vector< Vec4i> hierarchy; //int 자료형 네 개를 저장할 수 있는 OpenCV 벡터 클래스

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
        flip(frame, frame, 1); // 1:상하반전 
        if (pianoCnt == 1)
        {
            cvtColor(frame, lineMask, COLOR_BGR2GRAY);
            adaptiveThreshold(lineMask, lineMask, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 21, 5);

            //imshow("thres", lineMask);
            findContours(lineMask, con, RETR_TREE, CHAIN_APPROX_TC89_L1);
            pianoCnt++;
        }


        handImg = getHandMask(frame).clone(); //깊은복사

        //imshow("이진화", handImg);
        mask = handImg.clone(); // handImg를 mask 에 깊은복사
        findContours(mask, contours, hierarchy, RETR_LIST, CHAIN_APPROX_TC89_L1); //mask 처리된 이미지 소스(첫번째인자)로 부터 윤곽선을 찾고, contour 벡터 안에 Point 정보를 저장.
        // hierachy 세번째인자: 검출된 외곽선의 계층 정보가 저장됨. vector<Vec4i> 타입의 변수 지정.
        // RETR_LIST: 객체 바깥쪽과 안쪽 외곽선을 모두 검색, 계층 구조는 만들지 않음.
        //CHAIN_APPROX_TC89_KOS: Teh& Chin k cos 근사화를 적용합니다.
        int largestContour = 0;

        for (int k = 0; k < contours.size(); k++) {
            if (contourArea(contours[k]) > 6000) { // contourArea: 이미지에서 컨투어 영역을 얻는다. 
                largestContour = k; // 살색으로 검출된 가장 큰 영역을 손으로 인식하기위해
                drawContours(frame, contours, largestContour, Scalar(255, 210, 90), 2, 8, vector < Vec4i>(), 0, Point());
                //frame: 컨투어 라인을 그릴 이미지, contours: 컨투어해서 얻은 Point 집합 데이터, largestContour: 인덱스 지정 , 컨투어 색상, 컨투어 두께
                if (!contours.empty()) {
                    vector<vector<Point>>hull(1);
                    convexHull(Mat(contours[largestContour]), hull[0], false); //최외각선 검출 (여러개의 점들중에 최외각의 점들을 선으로 이어 볼록껍질을 만듬)
                    //contour: 윤곽선 좌표 (정점들)
                    // hull: 주어진 정점들을 볼록하게 감쌀 수 있는 정점 배열
                    drawContours(frame, hull, 0, Scalar(0, 255, 0));

                    if (hull[0].size() > 2) { //convexHull은 우선 스택에 첫번째정점과 두번째 정점에 해당하는 값을 스택에 넣고 시작

                        Rect boundingBox = boundingRect(hull[0]); //이미지 주위에 대략적 사각형을 그리는 함수
                        //rectangle(frame, boundingBox, Scalar(255), 2);
                        Point center = Point(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2); //손 중앙 

                        vector<Point> validPoints;

                        vector<int> hullIndex;
                        convexHull(Mat(contours[largestContour]), hullIndex);  //최외각선 검출 (여러개의 점들중에 최외각의 점들을 선으로 이어 볼록껍질을 만듬)
                        vector<Vec4i> convexityDefect; //Vec4i 클래스는 int 자료형 네 개를 저장할 수 있는 OpenCV 벡터 클래스

                        convexityDefects(Mat(contours[largestContour]), hullIndex, convexityDefect); //convexityDefect에 손가락 사이 움푹 파인 지점 ㅇ 반환.
                        //hullIndex: 볼록체를 만든 contour points의 인덱스를 포함한 convexHull을 인자로 받음.
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
                            double length = sqrt(pow(p1.x - p3.x, 2) + pow(p1.y - p3.y, 2)); //sprt 루트 , pow 제곱

                            //(angle > -30 && angle < 160 && abs(inA) > 10 && abs(inA) < 130 && length > 0.2 * boundingBox.height) 똑바른 손
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