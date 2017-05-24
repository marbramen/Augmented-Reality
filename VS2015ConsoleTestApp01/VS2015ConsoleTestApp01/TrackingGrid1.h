#ifndef TRACKINGGRID1_H
#define TRACKINGGRID1_H

#include <vector>
#include "opencv2/opencv.hpp"

class TrackingGrid1 {
public:
	int nPoints;
	TrackingGrid1(int);
	void sortBySecond(std::vector<std::pair<int, float> >&);	
	cv::Point2f getCentroide(std::vector<cv::Point2f>&);	
	std::vector<int> getPosCornes(std::vector<std::vector<cv::Point2f> >&);

};

#endif // TRACKINGGRID_H
