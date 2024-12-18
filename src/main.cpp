#include <iostream>
#include <chrono>

#include <fmt/core.h>

#include "CCLabler.h"

void drawBoundingBoxes(cv::Mat& img, std::vector<cv::Rect> boxes) {
    for (cv::Rect rect : boxes) {
        cv::rectangle(img, rect, cv::Scalar(0, 0, 255));
    }
}

int main() {
       
    cv::Mat img = cv::imread("../../../map.bmp");

    BinaryArray binaryMap = CCLabeler::imgToMap(img);
    auto start = std::chrono::high_resolution_clock::now();
    //std::vector<cv::Rect> bboxes = GraphTraverser::getBoundingBoxes(binaryMap);
    std::vector<cv::Rect> bboxes = LevialdiAlgorithm::getBoundingBoxes(binaryMap);
    int time = std::chrono::duration_cast<std::chrono::milliseconds>((std::chrono::high_resolution_clock::now() - start)).count();
    fmt::println("Run time - {}ms", time);

    drawBoundingBoxes(img, bboxes);

    cv::imshow("Input", img);
    cv::waitKey(0);

    return 0;
}