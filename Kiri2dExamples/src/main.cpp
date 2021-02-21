#include <stdio.h>
#include <opencv2/opencv.hpp>
#include<root_directory.h>
int main()
{
    printf("Hello\n");

    const std::string imagePath = std::string(RESOURCES_PATH) + "images/test.png";
    cv::Mat image = cv::imread(imagePath);
    cv::imshow("Display", image);

    cv::waitKey(0);
    return 0;
}