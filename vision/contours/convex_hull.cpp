#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>

using namespace cv;
using namespace std;

Mat frame_gray;
int thresh = 100;
RNG rng(12345);

void thresh_callback(int, void*);

int main(){
    Mat frame, frame_gray;
    VideoCapture cap(0);

    const char* source_window = "original image";
    namedWindow(source_window);
    createTrackbar("Canny thresh", source_window, &thresh, 255, thresh_callback);

    while(true){
        cap >> frame;

        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
        blur(frame_gray, frame_gray, Size(3,3));

        if (waitKey(1) == 27) break;
    }
}

void thresh_callback(int, void*){
    Mat canny_output;
    Canny(frame_gray, canny_output, thresh, thresh*2);

    vector<vector<Point> > contours;
    findContours(canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    vector<vector
}