#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(){
    Mat image = imread("lena.jpg");
    Mat new_image = Mat::zeros(image.size(), image.type());

    double alpha = 1.0;
    int beta = 0;

    cout << "alpha: "; cin >> alpha;
    cout << "beta: "; cin >> beta;

    for (int y = 0; y < image.rows; y++){
        for (int x = 0; x < image.cols; x++){
            for (int c = 0; c < image.channels(); c++){
                new_image.at<Vec3b>(y,x)[c] = saturate_cast<uchar>(alpha * image.at<Vec3b>(y,x)[c] + beta);
            }
        }
    }

    imshow("original", image);
    imshow("new image", new_image);
    waitKey();

    return 0;
}