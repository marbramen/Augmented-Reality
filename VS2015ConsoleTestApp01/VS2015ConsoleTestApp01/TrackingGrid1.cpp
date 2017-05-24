#ifndef TRACKINGGRID1_CPP
#define TRACKINGGRID1_CPP
#include "trackingGrid1.h"
#include "Geometria.h"
#include <set>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;
using namespace cv;

bool compareUpToDown(const pair<int, int>&i, const pair<int, int>&j) {
	return i.second < j.second;
}

TrackingGrid1::TrackingGrid1(int n) {
	nPoints = n;
}

void TrackingGrid1::sortBySecond(vector<pair<int, float> >& array) {
	sort(array.begin(), array.end(), compareUpToDown);	
}

Point2f TrackingGrid1::getCentroide(vector<Point2f>& array) {
	float x = 0, y = 0;
	int n = array.size();
	for (int i = 0; i < n; i++) {
		x += array[i].x;
		y += array[i].y;
	}
	return Point2f(x / n, y / n);
}

vector<int> TrackingGrid1::getPosCornes(vector<vector<Point2f> >& hull) {
	vector<int> posCornes;
	double piValue = 3.14159265, maxAngle = 148;
	int nHull = (int)hull[0].size();
	for (int i = 0; i < nHull; i++) {
		float angle;
		PointFrz A(hull[0][i].x - hull[0][(i == 0 ? nHull : i) - 1].x, hull[0][i].y - hull[0][(i == 0 ? nHull : i) - 1].y);
		PointFrz B(hull[0][i].x - hull[0][i == nHull - 1 ? 0 : i + 1].x, hull[0][i].y - hull[0][i == nHull - 1 ? 0 : i + 1].y);
		angle = atan2(cross(A, B), dot(A, B)) * 180 / piValue;
		if (abs(angle) <= maxAngle)
			posCornes.push_back(i);
	}
	return posCornes;
}


#endif // TRACKINGGRID1_CPP

