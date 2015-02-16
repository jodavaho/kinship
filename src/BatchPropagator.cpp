#include <Kinematics/BatchPropagator.h>
#include <stdio.h>
#include "math.h"

using namespace cv;
using namespace RSN;

void BatchPropagator::BatchPropagator::resetEstimates() {
	odometryEstimateVector.clear();
	odometryCovVector.clear();
	transformEstimateVector.clear();
	transformCovVector.clear();
}

Mat BatchPropagator::getCartEstimateFromOdom(Mat odometryEstimate) {
	float v = odometryEstimate.at<float>(0,0);
	float w = odometryEstimate.at<float>(1,0);
	Mat cartEstimate = (Mat_<float>(3,1) << v*sinc(w), v*cosc(w), w);
	return cartEstimate;
}

Mat BatchPropagator::getCartCovFromOdom(Mat odometryEstimate, Mat odometryCov) {
	float v = odometryEstimate.at<float>(0,0);
	float w = odometryEstimate.at<float>(1,0);
	Mat G = (Mat_<float>(3,3) << sinc(w),v*sinc_derivative(w),-sin(w),
								 cosc(w),v*cosc_derivative(w),cos(w),
								 0,	1, 0);
	Mat cartCov = G*odometryCov*G.t();
	return cartCov;
}

BatchPropagator::BatchPropagator(float width, float kv, float krl, float ks, float kwv, float kww) {
	assert(width>0.0001);
	this->width = width;
	this->kv = kv;
	this->krl = krl;
	this->ks = ks;
	this->kwv = kwv;
	this->kww = kww;
	resetEstimates();
}

BatchPropagator::BatchPropagator(float width, float wheel_var) {
	assert(width>0.0001);
	this->width = width;
	krl = wheel_var;
	kv = 0;
	ks = 0;
	kwv = 0;
	kww = 0;
	resetEstimates();
}

void BatchPropagator::addOdometryMeasurement(float left, float right, bool parallel) {
	float v = (left + right)/2;
	float w = (right - left)/width;

	float v_var = kv*kv*v*v + krl*krl*(left*left + right*right)/4 + SMALL_NUMBER;
	float w_var = krl*krl*(right*right + left*left)/(width*width) + kww*kww*w*w + kwv*kwv*v*v + SMALL_NUMBER;
	float s_var = ks*ks*v*v + SMALL_NUMBER;

	Mat newEstimate = (Mat_<float>(3,1) << v, w, 0);
	Mat newCov = (Mat_<float>(3,1) << v_var, w_var, s_var);
	newCov = Mat::diag(newCov);

	//cout << " w: " << w << "\n";
	//dumpMatrix(newCov);

	if (odometryEstimateVector.empty() || !parallel) {
		odometryEstimateVector.push_back(newEstimate);
		odometryCovVector.push_back(newCov);
	} else {
		Mat &oldEstimate = odometryEstimateVector.back();
		Mat &oldCov = odometryCovVector.back();
		mergeMeasurements(newEstimate, newCov, oldEstimate, oldCov, oldEstimate, oldCov);
	}
}

void BatchPropagator::addOdometryMeasurement(float v, float w, float dt, float v_var, float w_var, float s_var, bool parallel) {
	v = v*dt;
	w = w*dt;
	v_var *= dt;
	w_var *= dt;
	s_var *= dt;

	Mat newEstimate = (Mat_<float>(3,1) << v, w, 0);
	Mat newCov = (Mat_<float>(3,1) << v_var, w_var, s_var) + SMALL_NUMBER;
	newCov = Mat::diag(newCov);

	//dumpMatrix(newCov);

	if (odometryEstimateVector.empty() || !parallel) {
		odometryEstimateVector.push_back(newEstimate);
		odometryCovVector.push_back(newCov);
	} else {
		Mat &oldEstimate = odometryEstimateVector.back();
		Mat &oldCov = odometryCovVector.back();
		mergeMeasurements(newEstimate, newCov, oldEstimate, oldCov, oldEstimate, oldCov);
	}
}

void BatchPropagator::addTransformMeasurement(Mat localTransform, Mat cov, bool parallel) {
	cov = cov + SMALL_NUMBER;

	if (transformEstimateVector.empty() || !parallel) {
		transformEstimateVector.push_back(localTransform);
		transformCovVector.push_back(cov);
	} else {
		Mat &oldEstimate = transformEstimateVector.back();
		Mat &oldCov = transformCovVector.back();
		mergeMeasurements(localTransform, cov, oldEstimate, oldCov, oldEstimate, oldCov);
	}
}

/*
 * Do this at the end of the time step
 */
void BatchPropagator::propagate(Mat& position, Mat& cov) {

	Mat transformEstimate;
	Mat transformCov;

	if (!transformEstimateVector.empty()) {
		transformEstimate = transformEstimateVector.at(0);
		transformCov = transformCovVector.at(0);
		for (int i=1; i<transformEstimateVector.size(); i++) {
			addMeasurements(transformEstimate, transformCov, transformEstimateVector.at(i), transformCovVector.at(i), transformEstimate, transformCov);
		}
	}

	Mat odomEstimate;
	Mat odomCov;

	Mat cartOdomEstimate;
	Mat cartOdomCov;

	if (!odometryEstimateVector.empty()) {
		odomEstimate = odometryEstimateVector.at(0);
		odomCov = odometryCovVector.at(0);
		cartOdomEstimate = getCartEstimateFromOdom(odomEstimate);
		cartOdomCov = getCartCovFromOdom(odomEstimate, odomCov);
		for (int i=1; i<odometryEstimateVector.size(); i++) {
			odomEstimate = odometryEstimateVector.at(i);
			odomCov = odometryCovVector.at(i);
			addMeasurements(cartOdomEstimate, cartOdomCov, getCartEstimateFromOdom(odomEstimate), getCartCovFromOdom(odomEstimate, odomCov), cartOdomEstimate, cartOdomCov);
		}
	}

	if (transformEstimateVector.empty()) {
		transformEstimate = cartOdomEstimate;
		transformCov = cartOdomCov;
	} else if (odometryEstimateVector.empty()) {
		// nothing needs be done
	} else {
		mergeMeasurements(transformEstimate, transformCov, cartOdomEstimate, cartOdomCov, transformEstimate, transformCov);
	}

	addMeasurements(position, cov, transformEstimate, transformCov, position, cov);

	resetEstimates();
}
