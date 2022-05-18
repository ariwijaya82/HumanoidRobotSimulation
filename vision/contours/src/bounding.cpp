#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

Mat src_gray;
int thresh = 100;
RNG rng(12345);
const char* window_name = "image";
const char* contours_name = "contours";

void thresh_callback(int, void*){
    Mat canny_output;
    Canny(src_gray, canny_output, thresh, thresh*2);

    vector<vector<Point> >contours;
    findContours(canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    vector<Point2f> centers(contours.size());
    vector<float> radius(contours.size());

    for (size_t i = 0; i < contours.size(); i++){
        approxPolyDP(contours[i], contours_poly[i], 3, true);
        boundRect[i] = boundingRect(contours_poly[i]);
        minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
    }

    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);

    cout << contours.size() << endl;
    for (size_t i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        drawContours(drawing, contours_poly, (int)i, Scalar(255,255,255));
        rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), Scalar(255, 0, 0), 2);
        circle(drawing, centers[i], (int)radius[i], Scalar(0,0,255), 2);
    }
    // Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    // rectangle(drawing, boundRect[1].tl(), boundRect[1].br(), color, 2);
    // circle(drawing, centers[1], (int)radius[1], color, 2);

    imshow(contours_name, drawing);
}

int main(int argc, char** argv) {
    Mat src = imread("../data/cards.png");

    cvtColor(src, src_gray, COLOR_BGR2GRAY);
    blur(src_gray, src_gray, Size(3,3));

    namedWindow(window_name);
    imshow(window_name, src);

    const int max_thresh = 255;
    namedWindow(contours_name);
    createTrackbar("canny thresh", contours_name, &thresh, max_thresh, thresh_callback);
    thresh_callback(0,0);

    waitKey();
    return 0;
}