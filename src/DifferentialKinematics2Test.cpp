#include <opencv2/opencv.hpp>
#include <Kinematics/BatchPropagator.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

using namespace cv;
using namespace RSN;

float s_kv = 0;
float s_krl = 0.002;
float s_ks = 0.001;
float s_kw = 0.10;

int main(int argc, char** argv){
	Mat state = Mat::zeros(3,1,CV_32F);
	Mat cov = Mat::zeros(3,3,CV_32F);
	BatchPropagator robot = BatchPropagator(0.5, 0.1);
	robot.addOdometryMeasurement(1, 1, false);
	robot.propagate(state, cov);
	dumpMatrix(state);
	dumpMatrix(cov);
	robot.addOdometryMeasurement(0, 1, false);
	robot.propagate(state, cov);
	dumpMatrix(state);
	dumpMatrix(cov);
	state = Mat::zeros(3,1,CV_32F);
	cov = Mat::zeros(3,3,CV_32F);
	robot.addOdometryMeasurement(1, 1, false);
	robot.addOdometryMeasurement(0, 1, false);
	robot.propagate(state, cov);
	dumpMatrix(state);
	dumpMatrix(cov);
	state = Mat::zeros(3,1,CV_32F);
	cov = Mat::zeros(3,3,CV_32F);
	robot.addOdometryMeasurement(1, 1, false);
	Mat transform = (Mat_<float>(3,1,CV_32F) << 1, 0, 0.3);
	Mat transformCov = Mat::eye(3,3,CV_32F) * 0.01;
	robot.addTransformMeasurement(transform, transformCov, false);
	robot.propagate(state, cov);
	dumpMatrix(state);
	dumpMatrix(cov);
}
