
#include <fmt/core.h>

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

// Rotates array clockwise
BinaryArray rotate(const BinaryArray& input) {
    BinaryArray out(input.cols(), input.rows());
    for (int row = 0; row < input.rows(); row++) {
        for (int col = 0; col < input.cols(); col++) {
            out(col, row) = input(input.rows() - 1 - row, col);
        }
    }
    return out;
}

// Rotates point in img counterclockwise
cv::Point rotate(cv::Point p, int h) {
    return cv::Point(p.y, h - p.x);
}

/**
 * Finds bounding boxes of connected components by Levialdi's shrinking algorithm
 */
std::vector<cv::Rect> LevialdiAlgorithm::getBoundingBoxes(const BinaryArray& input) {
    std::vector<cv::Rect> boxes;

    std::vector<Corner> corners[4]; // bottomRight, topRight, topLeft, bottomLeft
    corners[0] = levialdiShrink(input);
    BinaryArray rotated = rotate(input);
    for (int i = 1; i < 4; i++) {
        corners[i] = levialdiShrink(rotated);
        rotated = rotate(rotated);
    }

    std::vector<CornerSet> cornerSets;
    for (int i = 0; i < 4; i++) {

        // Rotate corner points back to input space
        for (int j = 0; j < corners[0].size(); j++) {
            int w = input.cols() - 1;
            int h = input.rows() - 1;
            for (int k = 0; k < i; k++) {
                corners[i][j].point = rotate(corners[i][j].point, h);
                int temp = h;
                h = w;
                w = temp;
            }

            bool added = false;
            for (CornerSet& set : cornerSets) {
                if (set.antiDiagonal && (i == 0 || i == 2)) continue;
                if (!set.antiDiagonal && (i == 1 || i == 3)) continue;
                if (corners[i][j].time == set.time) {
                    if (i == 0 || i == 3) {
                        set.lower.push_back(corners[i][j].point);
                    } else {
                        set.upper.push_back(corners[i][j].point);
                    }
                    added = true;
                    break;
                }
            }
            if (!added) {
                CornerSet set(corners[i][j].time);
                if (i == 0 || i == 3) {
                    set.lower.push_back(corners[i][j].point);
                }
                else {
                    set.upper.push_back(corners[i][j].point);
                }
                set.antiDiagonal = i == 1 || i == 3;
                cornerSets.push_back(set);
            }
        }
    }

    // TODO: remove duplicates
    for (CornerSet& set : cornerSets) {
        if (set.upper.size() != set.lower.size()) {
            continue;
        }
        if (set.lower.size() == 1) {
            cv::Point ul;
            cv::Point lr;
            if (set.antiDiagonal) {
                ul = cv::Point(set.lower[0].x, set.upper[0].y);
                lr = cv::Point(set.upper[0].x, set.lower[0].y);
            }
            else {
                ul = set.upper[0];
                lr = set.lower[0];
            }
            boxes.push_back(cv::Rect(cv::Point(ul.x - 1, ul.y - 1), cv::Point(lr.x + 1, lr.y + 1)));
        } else {
            // TODO
            std::cout << "Congradulations! You found an edge case!\n";
        }
    }

    return boxes;
}