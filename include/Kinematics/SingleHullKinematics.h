/*
 * SINGLEHULLKINEMATICS_H_
 *
 *  Created on: Feb 12, 2011
 *      Author: joshua vander hook
 */

#ifndef SINGLEHULLKINEMATICS_H_
#define SINGLEHULLKINEMATICS_H_
#define _USE_MATH_DEFINES
#include <opencv2/opencv.hpp>
#include <math.h>
#include <helpers/opencv.h>
#include <helpers/math.h>

namespace RSN{
typedef struct GPS_Measurement {
	/**
	 * 2x1 vector
	 */
	cv::Mat position;
	/**
	 * Row-wise 2x2 covariance of position
	 */
	cv::Mat cov;
	/**
	 * in degrees from north (compass frame)
	 */
	float track;
	/**
	 * in degrees
	 */
	float err_track;
	/**
	 * meters per second
	 */
	float speed;
	/**
	 * meters per second
	 */
	float err_speed;
} GPS_Measurement;

typedef struct SingleHullState {
	int sz;
	cv::Mat state;
	cv::Mat cov;
} SingleHullState;

typedef struct SingleHullConstants {

	float Mx;
	float My;
	float Ir;

	float Kx;
	float Ky;
	float Kr;

	float dp;

	float Kprop;

	float s_th;
	float s_an;
	float mn_th;

} SingleHullConstants;

/**
 * See :
 * Caccia, Bibuli, Bono, Bruzzone 2008
 * Springer
 * DOI 10.1007/s10514-008-9100-0
 * for dicussion
 */
class SingleHullKinematics {

public:
	SingleHullKinematics();
	SingleHullKinematics(SingleHullConstants);
	~SingleHullKinematics();

	/**
	 * predict step. Taking into account constant thrust and angle for time step t.
	 * uses more accurate model
	 */
	void ekfPredict(float thrust, float angle, float time, SingleHullState& state);
	void ekfPredict(float thrust, float angle, float time, cv::Mat&pose, cv::Mat& out_cv);
	/**
	 * Internal, uses approx model
	 */
	void ekfPredictSimple(float thrust, float angle, float time, cv::Mat&pose, cv::Mat& out_cv);
	/**
	 * Internal, uses more accurate model
	 */
	void ekfPredictActual(float thrust, float angle, float time, cv::Mat&pose, cv::Mat& out_cv);
	void compassMeasurement(float degrees,float sigma, SingleHullState&state);
	void localGPSMeasure(GPS_Measurement Z,SingleHullState& X);
	//void gpsMeasurement(cv::Mat&X, cv::Mat&Q, cv::Mat&gps, cv::Mat&sig_gps);

	/**
	 * Generic Measurement
	 */
	void ekfMeasurement(cv::Mat&X, cv::Mat&hofX,cv::Mat H, cv::Mat&Q,cv::Mat&Z,cv::Mat&R);
	/**
	 * Pure predict (noiseless) for time step t
	 */
	void updatePose(float thrust, float angle, float time, cv::Mat& out);
	/**
	 * Model-based acceleration
	 */
	void getAccVectors(float thrust,float angle,cv::Mat pose, cv::Mat out_acc);

	SingleHullConstants getConstants();
	void setConstants(SingleHullConstants c);

private:
	//these methods used internally

	SingleHullConstants cc;
	SingleHullState state;
};
}
#endif /* SINGLEHULLKINEMATICS_H_ */
