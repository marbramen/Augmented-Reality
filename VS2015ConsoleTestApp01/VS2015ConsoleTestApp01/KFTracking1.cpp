#ifndef KFTRACKING1_CPP
#define KFTRACKING1_CPP

#include "KFTracking1.h"
#include <opencv2/video/tracking.hpp>
#include <vector>

using namespace std;
using namespace cv;

KFTracking1::KFTracking1(int n) {
	stateSize = STATE_SIZE;
	measSize = MEASURE_SIZE;
	contrSize = CONTROL_SIZE;
	type = CV_32F;

	firstFound.clear();
	for (int i = 0; i < n; i++)
		firstFound.push_back(true);

	KalmanFilter kf(stateSize, measSize, contrSize, type);

	// State Matrix
	// [px py vx vy hx wx]'

	// Transition Matrix A
	// Note: set dT at each processing step!

	// Note: set dT at each processing step!
	// [ 1 0 dT 0  0 0 ]
	// [ 0 1 0  dT 0 0 ]
	// [ 0 0 1  0  0 0 ]
	// [ 0 0 0  1  0 0 ]
	// [ 0 0 0  0  1 0 ]
	// [ 0 0 0  0  0 1 ]
	setIdentity(kf.transitionMatrix);
	kf.transitionMatrix.at<float>(2) = 1e-2f;
	kf.transitionMatrix.at<float>(9) = 1e-2f;

	// Measure Matrix H
	// [ 1 0 0 0 0 0 ]
	// [ 0 1 0 0 0 0 ]
	// [ 0 0 0 0 1 0 ]
	// [ 0 0 0 0 0 1 ]
	kf.measurementMatrix.at<float>(0) = 1.0f;
	kf.measurementMatrix.at<float>(7) = 1.0f;
	kf.measurementMatrix.at<float>(16) = 1.0f;
	kf.measurementMatrix.at<float>(23) = 1.0f;

	// Process Noise Covariance Matrix Q
	//    [ Ex 0  0    0 0    0 ]
	//    [ 0  Ey 0    0 0    0 ]
	//    [ 0  0  Ev_x 0 0    0 ]
	//    [ 0  0  0    1 Ev_y 0 ]
	//    [ 0  0  0    0 1    Ew ]
	//    [ 0  0  0    0 0    Eh ]
	kf.processNoiseCov.at<float>(0) = 1e-2;
	kf.processNoiseCov.at<float>(7) = 1e-2;
	kf.processNoiseCov.at<float>(14) = 2.0f;
	kf.processNoiseCov.at<float>(21) = 1.0f;
	kf.processNoiseCov.at<float>(28) = 1e-2;
	kf.processNoiseCov.at<float>(35) = 1e-2;

	// Measurement Noise Covariance Matrix R
	// [Zpx2 0]
	// [0 Zpy2]
	setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
	setIdentity(kf.errorCovPre, Scalar(1));
	setIdentity(kf.errorCovPost, Scalar::all(1));

	arrayKF = new vector<KalmanFilter>();
	for (int i = 0; i < n; i++) {
		(*arrayKF).push_back(kf);
	}
}

KFTracking1::~KFTracking1() {
	delete arrayKF;
}

void KFTracking1::setStateInit(vector<Mat>* arrayMat) {
	for (int i = 0; i < (int)arrayMat->size(); i++) {
		setIdentity((*arrayKF)[i].errorCovPre);
		(*arrayKF)[i].statePost.at<float>(0) = (*arrayMat)[i].at<float>(0);
		(*arrayKF)[i].statePost.at<float>(1) = (*arrayMat)[i].at<float>(1);
		(*arrayKF)[i].statePost.at<float>(4) = (*arrayMat)[i].at<float>(2);
		(*arrayKF)[i].statePost.at<float>(5) = (*arrayMat)[i].at<float>(3);
		(*arrayKF)[i].statePost.at<float>(2) = 0;
		(*arrayKF)[i].statePost.at<float>(3) = 0;
	}
}

vector<Mat>* KFTracking1::kalmanCorrection(vector<Mat>* arrayMat) {
	vector<Mat>* correct = new vector<Mat>();
	for (int i = 0; i < (int)arrayMat->size(); i++) {
		correct->push_back((*arrayKF)[i].correct((*arrayMat)[i]));
	}
	return correct;
}

vector<Mat>* KFTracking1::predict(double dt) {
	vector<Mat>* predic = new vector<Mat>();
	for (int i = 0; i < (int)(*arrayKF).size(); i++) {
		(*arrayKF)[i].transitionMatrix.at<float>(2) = dt;
		(*arrayKF)[i].transitionMatrix.at<float>(9) = dt;
		predic->push_back((*arrayKF)[i].predict());
	}
	return predic;
}

vector<Mat>* KFTracking1::futureNTime(int jt) {
	vector<Mat>* res = new vector<Mat>();
	for (int i = 0; i < (int)(*arrayKF).size(); i++) {
		KalmanFilter kfTemp = (*arrayKF)[i];
		for (int j = 0; j < jt; j++) {
			kfTemp.statePost = kfTemp.statePre;
			kfTemp.predict();
		}
		res->push_back(kfTemp.statePre);
	}
	return res;
}

#endif //KFTRACKING1_CPP
