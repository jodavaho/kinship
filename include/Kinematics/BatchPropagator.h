
#ifndef BATCHPROPAGATOR_H_
#define BATCHPROPAGATOR_H_

#include <opencv2/opencv.hpp>
#include <helpers/opencv.h>
#include <helpers/math.h>
#include <math.h>
#include <vector>

namespace RSN{

const float SMALL_NUMBER = pow(10,-6); // Variance when starting from a known position.
const float BIG_NUMBER = pow(10,6); // Variance of a measurement that isn't real.

/**
 * Constructs a rotation matrix for you.
 */
void inline getRot1(cv::Mat& out, float theta){
	out.at<float>(0,0)=cos(theta);
	out.at<float>(0,1)=-sin(theta);
	out.at<float>(1,0)=sin(theta);
	out.at<float>(1,1)=cos(theta);
}

/**
 * Constructs the derivative of the rotation matrix w/r/t theta
 */
void inline getRotPrime(cv::Mat& out, float theta){
	out.at<float>(0,0)=-sin(theta);
	out.at<float>(0,1)=cos(theta);
	out.at<float>(1,0)=cos(theta);
	out.at<float>(1,1)=-sin(theta);
}
/**
 * Constructs a transformation matrix, given a displacement vector and a rotation matrix
 */
/*
cv::Mat inline getT1(cv::Mat& R, cv::Mat& T){
	cv::Mat ret = cv::Mat::eye(3,3,CV_32F);
	cv::Mat rslice = cv::Mat(ret,cv::Range(0,2),cv::Range(0,2));
	cv::Mat tslice = cv::Mat(ret,cv::Range(0,2),cv::Range(2,3));
	R.copyTo(rslice);
	T.copyTo(tslice);
	return ret;
}
*/
/**
 * This combines two estimates of any type
 * This can be described as an update step
 */
void inline mergeMeasurements(cv::Mat estimate1, cv::Mat cov1, cv::Mat estimate2, cv::Mat cov2, cv::Mat& estimateOut, cv::Mat& covOut) {
	estimateOut = (cov1+cov2).inv()*(cov1*estimate2 + cov2*estimate1);
	covOut = (cov1.inv() + cov2.inv()).inv();
}

/**
 * This adds two uncertain, cartesian transformations into a single transformation
 * This can be described as a propagation step
 */
void inline addMeasurements(cv::Mat estimate1, cv::Mat cov1, cv::Mat estimate2, cv::Mat cov2, cv::Mat& estimateOut, cv::Mat& covOut) {
	float phi1 = estimate1.at<float>(2,0);
	cv::Mat R = cv::Mat::eye(3,3,CV_32F);
	getRot1(R,phi1);
	cv::Mat rotatedEst2 = R*estimate2;

	cv::Mat Rp = cv::Mat::zeros(3,3,CV_32F);
	getRotPrime(Rp,phi1);
	cv::Mat del_phi = Rp*estimate2;
	// not 100% sure if this should be estimate2 or rotatedEst2...
	del_phi.at<float>(0,2) = 1;

	cv::Mat PHI = cv::Mat::eye(3,3,CV_32F);
	PHI.at<float>(0,2) = del_phi.at<float>(0,0);
	PHI.at<float>(1,2) = del_phi.at<float>(0,1);
	PHI.at<float>(2,2) = del_phi.at<float>(0,2);

	estimateOut = estimate1 + rotatedEst2;
	covOut = PHI*cov1*PHI.t() + cov2;
}

float inline sinc(float x){
	if (abs(x) <= SMALL_NUMBER) return 1;
	return sin(x)/x;
}

float inline cosc(float x){
	if (abs(x) <= SMALL_NUMBER) return 0;
	return (1-cos(x))/x;
}

float inline sinc_derivative(float x){
	if (abs(x) <= SMALL_NUMBER) return 0;
	return (x*cos(x) - sin(x))/(x*x);
}

float inline cosc_derivative(float x){
	if (abs(x) <= SMALL_NUMBER) return 1/2;
	return (x*sin(x) + cos(x) - 1)/(x*x);
}

class BatchPropagator {
public:
	float width;
	float kv; // chassis speed sigma proportional to chassis speed
	float krl; // wheel speed sigma proportional to wheel speed
	float ks; // sideways chassis slip sigma proportional to chassis speed
	float kwv; // angular speed sigma proportional to chassis speed
	float kww; // angular speed sigma proportional to angular speed
	BatchPropagator(float width, float kv, float krl, float ks, float kwv, float kww);
	BatchPropagator(float width, float krl); // for purists
	void addOdometryMeasurement(float left, float right, bool parallel);
	void addOdometryMeasurement(float v, float w, float dt, float v_var, float w_var, float s_var, bool parallel);
	void addTransformMeasurement(cv::Mat localTransform, cv::Mat cov, bool parallel);
	// parallel means in parallel with the *previous* measurement of the same type.
	void propagate(cv::Mat &position, cv::Mat &cov);
	void resetEstimates();
private:
	std::vector<cv::Mat> odometryEstimateVector;
	std::vector<cv::Mat> odometryCovVector;
	std::vector<cv::Mat> transformEstimateVector;
	std::vector<cv::Mat> transformCovVector;
	cv::Mat getCartEstimateFromOdom(cv::Mat odometryEstimate);
	cv::Mat getCartCovFromOdom(cv::Mat odometryEstimate, cv::Mat odometryCov);
};
}

#endif
