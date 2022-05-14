#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;

int main(int argc, char** argv) {
    Mat src, dst, kernel;
    Point anchor;
    double delta;
    int ddepth;
    int kernel_size;

    const char* window_name = "filter2D Demo";
    src = imread("lena.jpg", IMREAD_COLOR);
    if (src.empty()) {
        printf("Could not open or find image\n");
        return EXIT_FAILURE;
    }

    anchor = Point(-1, -1);
    delta = 0;
    ddepth = -1;

    int ind = 0;
    for (;;) {
        kernel_size = 3 + 2*(ind % 5);
        kernel = Mat::ones(kernel_size, kernel_size, CV_32F) / (float)(kernel_size*kernel_size);

        filter2D(src, dst, ddepth, kernel, anchor, delta, BORDER_DEFAULT);
        imshow(window_name, dst);

        char c = (char) waitKey(500);
        if (c == 27) break;
        ind++;
    }

    return EXIT_SUCCESS;
}