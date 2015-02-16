/*
 * DifferentialKinematics.h
 *
 *  Created on: Feb 2, 2011
 *      Author: joshua
 */

#ifndef DIFFERENTIALKINEMATICS_H_
#define DIFFERENTIALKINEMATICS_H_

//for fast matrix multiplications, etc.
#include <Eigen/Dense>
#include <helpers/math.h>
#include <math.h>

namespace RSN{
/**
 * Constructs a rotation matrix for you.
 */
void inline getRot(Eigen::MatrixXd &out, float theta){
	out(0,0)=cos(theta);
	out(0,1)=-sin(theta);
	out(1,0)=sin(theta);
	out(1,1)=cos(theta);
}

/**
 * Returns the angle of the rotation matrix
 */
float inline getTheta(Eigen::MatrixXd& R){
	return atan2(R(1,0),R(0,0));
}

class DifferentialKinematics {
public:
	DifferentialKinematics(float width_of_chassis);
	virtual ~DifferentialKinematics();
	Eigen::MatrixXd getWheelSpeeds(Eigen::MatrixXd& velocities);
	Eigen::MatrixXd getWheelSpeeds(float linear, float angular);
	Eigen::MatrixXd getVelocities(Eigen::MatrixXd& wheelspeeds);
	void getExpectedTransformFromOrderedVelocities(float linear, float angular, float deltaT, Eigen::MatrixXd& out_Transformation);
	void getTransformationFromDistances(Eigen::MatrixXd& dists, Eigen::MatrixXd& out_Transformation);
	void getTransformationFromDistances(float left, float right, Eigen::MatrixXd& out_Transformation);
	float getTheta(float left, float right);
	float getVel(float left, float right);
	void ekfPredict(float left, float right,Eigen::MatrixXd&pose, Eigen::MatrixXd& out_cv);
	void ekfMeasurement(Eigen::MatrixXd&X,Eigen::MatrixXd&Q,Eigen::MatrixXd&Z,Eigen::MatrixXd&R);
	void updatePose(float left, float right, Eigen::MatrixXd& out);
	float kl;
	float kr;
	//void getTransformationFromDistances(float left, float right, Eigen::MatrixXd& out_R, Eigen::MatrixXd& out_T);
private:
	float width;
	Eigen::MatrixXd J; //forward
	Eigen::MatrixXd J2; //reverse
};
}
#endif /* DIFFERENTIALKINEMATICS_H_ */
