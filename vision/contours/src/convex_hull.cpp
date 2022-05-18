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
const char* source_window;

void thresh_callback(int, void*){
    if (thresh < 10){
        thresh = 10;
        setTrackbarPos("Canny thresh", source_window, thresh);
    }
};

int main(){
    Mat frame, frame_gray;
    VideoCapture cap(2);

    source_window = "original image";
    namedWindow(source_window);
    createTrackbar("Canny thresh", source_window, &thresh, 255, thresh_callback);
    thresh_callback(0,0);

    while(true){
        cap >> frame;

        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
        blur(frame_gray, frame_gray, Size(3,3)); 

        Mat canny_output;
        Canny(frame_gray, canny_output, thresh, thresh*2);

        vector<vector<Point> > contours;
        findContours(canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

        vector<vector<Point> >hull(contours.size());
        for (size_t i = 0; i < contours.size(); i++) {
            convexHull(contours[i], hull[i]);
        }

        Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
        for (size_t i = 0; i < contours.size(); i++){
            Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            drawContours(drawing, contours, (int)i, color);
            drawContours(drawing, hull, (int)i, color);
        }

        imshow(source_window, drawing);
        if (waitKey(1) == 27) break;
    }
}