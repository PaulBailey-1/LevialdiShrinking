#pragma once 

#include "CCLabler.h"

struct Corner {
	cv::Point point;
	int time;
	Corner() {}
	Corner(cv::Point p, int t) {
		point = p;
		time = t;
	}
};

std::vector<Corner> levialdiShrink(const BinaryArray& input);
