#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

void sharpen(const Mat& myImage, Mat& Result){
    CV_Assert(myImage.depth() == CV_8U);

    const int nChannels = myImage.channels();
    Result.create(myImage.size(), myImage.type());

    for (int j = 1; j < myImage.rows-1; j++) {
        const uchar* previous = myImage.ptr<uchar>(j - 1);
        const uchar* current  = myImage.ptr<uchar>(j);
        const uchar* next     = myImage.ptr<uchar>(j + 1);

        uchar* output = Result.ptr<uchar>(j);

        for (int i = nChannels; i < nChannels*(myImage.cols-1); i++) {
            *output++ = saturate_cast<uchar>(5*current[i]
                    - current[i-nChannels] - current[i+nChannels] - previous[i] - next[i]);
        }
    }

    Result.row(0).setTo(Scalar(0));
    Result.row(Result.rows-1).setTo(Scalar(0));
    Result.col(0).setTo(Scalar(0));
    Result.col(Result.cols-1).setTo(Scalar(0));
}

int main() {
    Mat src = imread("lena.jpg", IMREAD_COLOR);
    Mat dst, dst1;

    double t = (double)getTickCount();
    sharpen(src, dst);
    t = ((double)getTickCount() - t)/getTickFrequency();
    cout << "time ellapse: " << t << endl;
    imshow("image", src);
    imshow("filter", dst);
    waitKey();

    Mat kernel = (Mat_<char>(3,3) <<  0, -1,  0,
                                -1,  5, -1,
                                0, -1,  0);

    t = (double)getTickCount();
    cout << kernel << endl;
    filter2D( src, dst1, src.depth(), kernel );
    t = ((double)getTickCount() - t)/getTickFrequency();
    cout << "time ellapse: " << t << endl;
    imshow("filter", dst1);
    waitKey();
    return EXIT_SUCCESS;
}