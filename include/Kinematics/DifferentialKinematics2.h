/*
 * Created on: April 10, 2012
 * Author: Patrick
 */

#ifndef DIFFERENTIALKINEMATICS2_H_
#define DIFFERENTIALKINEMATICS2_H_

#include <opencv2/opencv.hpp>
#include <helpers/opencv.h>
#include <helpers/math.h>
#include <math.h>

namespace RSN{

const float SMALL = pow(10,-10);
const float BIG = pow(10,10); // I use this as covariance instead of infinity to improve computability.

/**
 * Constructs a rotation matrix for you.
 */
void inline getRot(cv::Mat &out, float theta){
	out.at<float>(0,0)=cos(theta);
	out.at<float>(0,1)=-sin(theta);
	out.at<float>(1,0)=sin(theta);
	out.at<float>(1,1)=cos(theta);
}

/**
 * Constructs the derivative of the rotation matrix w/r/t theta
 */
void inline getRotPrime(cv::Mat &out, float theta){
	out.at<float>(0,0)=-sin(theta);
	out.at<float>(0,1)=cos(theta);
	out.at<float>(1,0)=cos(theta);
	out.at<float>(1,1)=-sin(theta);
}

/**
 * Constructs a transformation matrix, given a displacement vector and a rotation matrix
 */
cv::Mat inline getT(cv::Mat& R, cv::Mat& T){
	cv::Mat ret = cv::Mat::eye(3,3,CV_32F);
	cv::Mat rslice = cv::Mat(ret,cv::Range(0,2),cv::Range(0,2));
	cv::Mat tslice = cv::Mat(ret,cv::Range(0,2),cv::Range(2,3));
	R.copyTo(rslice);
	T.copyTo(tslice);
	return ret;
}
/*
 * sin(x)/x
 */
float inline sinc(float x){
	if (abs(x) <= SMALL) return 1;
	return sin(x)/x;
}
/*
 * (1-cos(x))/x
 */
float inline cosc(float x){
	if (abs(x) <= SMALL) return 0;
	return (1-cos(x))/x;
}

float inline sinc_derivative(float x){
	if (abs(x) <= SMALL) return 0;
	return (x*cos(x) - sin(x))/(x*x);
}

float inline cosc_derivative(float x){
	if (abs(x) <= SMALL) return 1/2;
	return (x*sin(x) + cos(x) - 1)/(x*x);
}

class CartesianMeasurement {
public:
	cv::Mat vector;
	cv::Mat cov;
	CartesianMeasurement();
	CartesianMeasurement(cv::Mat vector, cv::Mat cov);
	CartesianMeasurement(float x, float y, float phi, cv::Mat cov);
	CartesianMeasurement(CartesianMeasurement* m1, CartesianMeasurement* m2);
};

class OdometryMeasurement {
public:
	float v;
	float w;
	float v_var;
	float w_var;
	float s_var;
	OdometryMeasurement();
	OdometryMeasurement(float v, float w, float v_var, float w_var, float s_var);
	OdometryMeasurement(OdometryMeasurement* m1, OdometryMeasurement* m2);
	CartesianMeasurement convertToCartesian();
private:
	cv::Mat convertToVector();
	cv::Mat getJacobian();
	cv::Mat getCartesianCovariance();
};

class DifferentialKinematics2 {
public:
	float a;
	float odom_v_var;
	float odom_s_var;
	float odom_w_var;
	CartesianMeasurement state;
	DifferentialKinematics2(float a, cv::Mat state_vector, cv::Mat cov, float odom_v_var, float odom_w_var, float odom_s_var);
	DifferentialKinematics2(float a, float odom_v_var, float odom_w_var, float odom_s_var);
	void addOdometryMeasurement(float left, float right);
	void addOdometryMeasurement(float v, float w, float dt, float v_var, float w_var, float s_var);
	void addTransformMeasurement(cv::Mat localState, cv::Mat cov);
	void addGlobalUpdate(cv::Mat globalState, cv::Mat cov);
	CartesianMeasurement performEKF();
private:
	OdometryMeasurement odom;
	CartesianMeasurement transform;
	CartesianMeasurement updateState;
};

}
#endif /* DIFFERENTIALKINEMATICS2_H_ */
