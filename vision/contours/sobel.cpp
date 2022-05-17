#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char** argv){
    Mat image, src, src_gray;
    Mat grad;

    const String window_name = "Sobel Demo";
    int ksize = 1;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    String imageName = "lena.jpg";
    image = imread(imageName, IMREAD_COLOR);

    FOR (;;){
        GaussianBlur(image, src, Size(3,3), 0, 0, BORDER_DEFAULT);
        cvtColor(src, src_gray, COLOR_BGR2GRAY);

        Mat grad_x, grad_y;
        Mat abs_grad_x, abs_grad_y;

        Sobel(src_gray, grad_x, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT);
        Sobel(src_gray, grad_y, ddepth, 0, 1, ksize, scale, delta, BORDER_DEFAULT);

        convertScaleAbs(grad_x, abs_grad_x);
        convertScaleAbs(grad_y, abs_grad_y);
    }
}