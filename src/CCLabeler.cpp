
#include "CCLabler.h"
#include "Levialdi.h"

BinaryArray CCLabeler::imgToMap(const cv::Mat& img) {
    BinaryArray mat(img.rows, img.cols);
    for (int y = 0; y < img.rows; y++) {
        for (int x = 0; x < img.cols; x++) {
            mat(y, x) = img.at<cv::Vec3b>(cv::Point(x, y))[0] == 0;
        }
    }
    return mat;
}

/**
 * Finds bounding boxes of connected components by DFS graph traversal
 *
 * @param input image
 * @return vector of rectangles as bounding boxes
 */
std::vector<cv::Rect> GraphTraverser::getBoundingBoxes(const BinaryArray& map) {
    BinaryArray mapCopy = map;
    std::vector<cv::Rect> boxes;
    std::stack<cv::Point> nodesToLabel;
    for (int row = 0; row < map.rows(); row++) {
        for (int col = 0; col < map.cols(); col++) {
            cv::Point point(col, row);
            if (map(row, col)) {
                getNeighbors(mapCopy, point, nodesToLabel);

                cv::Point boxMin(point);
                cv::Point boxMax(point);
                while (!nodesToLabel.empty()) {
                    cv::Point nextNode = nodesToLabel.top();
                    nodesToLabel.pop();
                    getNeighbors(mapCopy, nextNode, nodesToLabel);

                    if (nextNode.x < boxMin.x) boxMin.x = nextNode.x;
                    if (nextNode.y < boxMin.y) boxMin.y = nextNode.y;
                    if (nextNode.x > boxMax.x) boxMax.x = nextNode.x + 1;
                    if (nextNode.y > boxMax.y) boxMax.y = nextNode.y + 1;
                }
                boxes.push_back(cv::Rect(boxMin, boxMax));
            }
        }
    }
    return boxes;
}

void GraphTraverser::getNeighbors(BinaryArray& input, cv::Point point, std::stack<cv::Point>& remaining) {
    input(point.y, point.x) = false;
    for (int i = -1; i < 2; i++) {
        for (int j = -1; j < 2; j++) {
            if (point.x + i < 0 || point.x + i >= input.cols()) continue;
            if (point.y + j < 0 || point.y + j >= input.rows()) continue;
            cv::Point neighbor(point.x + i, point.y + j);
            if (input(neighbor.y, neighbor.x)) {
                remaining.push(neighbor);
            }
        }
    }
}

/**
 * Finds bounding boxes of connected components by Levialdi's shrinking algorithm
 *
 * @param input image
 * @return vector of rectangles as bounding boxes
 */
std::vector<cv::Rect> LevialdiAlgorithm::getBoundingBoxes(const BinaryArray& input) {
    std::vector<cv::Rect> boxes;

    std::vector<Corner> bottomRightCorners = levialdiShrink(input);

     //Doesn't work on non square arrays?
    BinaryArray flipped(input.rows(), input.cols());
    int w = input.cols() - 1;
    int h = input.rows() - 1;
    for (int row = 0; row < input.rows(); row++) {
        for (int col = 0; col < input.cols(); col++) {
            flipped(w - col, h - row) = input(row, col);
        }
    }
    std::vector<Corner> topLeftCorners = levialdiShrink(flipped);

    std::vector<CornerSet> cornerSets;
    for (int i = 0; i < topLeftCorners.size(); i++) {

        topLeftCorners[i].point = cv::Point(h - topLeftCorners[i].point.y, w - topLeftCorners[i].point.x);

        bool tlAdded = false;
        for (CornerSet& set : cornerSets) {
            if (std::abs(topLeftCorners[i].time - set.time) <= 50) {
                set.upperLefts.push_back(topLeftCorners[i].point);
                tlAdded = true;
                break;
            }
        }
        if (!tlAdded) {
            CornerSet set(topLeftCorners[i].time);
            set.upperLefts.push_back(topLeftCorners[i].point);
            cornerSets.push_back(set);
        }
    }
    for (int i = 0; i < bottomRightCorners.size(); i++) {
        bool brAdded = false;
        for (CornerSet& set : cornerSets) {
            if (std::abs(bottomRightCorners[i].time - set.time) <= 50) {
                set.lowerRights.push_back(bottomRightCorners[i].point);
                break;
            }
        }
    }
    for (CornerSet& set : cornerSets) {
        if (set.lowerRights.size() != set.upperLefts.size()) {
            std::cout << "Mismatched corner set\n";
        }
        if (set.lowerRights.size() == 1) {
            cv::Point ul = set.upperLefts[0];
            cv::Point lr = set.lowerRights[0];
            boxes.push_back(cv::Rect(cv::Point(ul.x - 1, ul.y - 1), cv::Point(lr.x + 1, lr.y + 1)));
        } else {
            // TODO
            std::cout << "Congradulations! You found an edge case!\n";
        }
    }

    return boxes;
}

//bool LevialdiAlgorithm::levialdiShrinkingOperator(const BinaryArray& input, BinaryArray& output, std::vector<Corner>& corners, int step, bool reverse) {
//    for (int rawRow = 1; rawRow < input.rows() - 1; rawRow++) {
//        int row = rawRow;
//        if (reverse) row = input.rows() - rawRow - 1;
//        for (int rawCol = 1; rawCol < input.cols() - 1; rawCol++) {
//            int col = rawCol;
//            if (reverse) col = input.cols() - rawCol - 1;
//            int offset = reverse ? 1 : -1;
//            if (input(row, col)) { // Case 1  
//                output(row, col) = input(row + offset, col) || input(row, col + offset) || input(row + offset, col + offset);
//            } else { // Case 2
//                output(row, col) = input(row + offset, col) && input(row, col + offset);
//            }
//        }
//    }
//    bool notDone = false;
//    for (int rawRow = 1; rawRow < input.rows() - 1; rawRow++) {
//        int row = rawRow;
//        if (reverse) row = input.rows() - rawRow - 1;
//        for (int rawCol = 1; rawCol < input.cols() - 1; rawCol++) {
//            int col = rawCol;
//            if (reverse) col = input.cols() - rawCol - 1;
//            int offset = reverse ? -1 : 1;
//            if (output(row, col)) {
//                notDone = true;
//            }
//            else if (input(row, col) && !output(row + offset, col) && !output(row, col + offset) && !output(row + offset, col + offset)) {
//                if (reverse) {
//                    corners.push_back(Corner(cv::Point(col - 1, row - 1), step));
//                } else {
//                    corners.push_back(Corner(cv::Point(col, row), step));
//                }
//            }
//        }
//    }
//    return notDone;
//}