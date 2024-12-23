#pragma once

#include <vector>
#include <list>
#include <stack>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

typedef Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> BinaryArray;

class CCLabeler {
public:
	static std::vector<cv::Rect> getBoundingBoxes(const BinaryArray& map);
	static BinaryArray imgToMap(const cv::Mat& img);
};

class GraphTraverser : public CCLabeler {
public:
	static std::vector<cv::Rect> getBoundingBoxes(const BinaryArray& map);
private:
	static void getNeighbors(BinaryArray& input, cv::Point point, std::stack<cv::Point>& remaining);
};

class LevialdiAlgorithm : public CCLabeler {

public:

	struct CornerSet {
		int time;
		std::vector<cv::Point> upperLefts;
		std::vector<cv::Point> lowerRights;
		CornerSet(int t) { time = t; }
	};

	static std::vector<cv::Rect> getBoundingBoxes(const BinaryArray& map);
private:
	//static bool levialdiShrinkingOperator(const BinaryArray& in, BinaryArray& out, std::vector<Corner>& corners, int step, bool reverse = false);
};