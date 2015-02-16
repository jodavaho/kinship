/*
 * DifferentialKinematics.cpp
 *
 *  Created on: Feb 2, 2011
 *      Author: joshua
 */

#include "Kinematics/DifferentialKinematics.h"
#include <stdio.h>
#include "math.h"

using namespace RSN;

DifferentialKinematics::DifferentialKinematics(float width_of_chassis) {
	assert(width_of_chassis>0.0001);
	width = width_of_chassis;
	J2=Eigen::MatrixXd(2,2);
	J2(0)=1;
	J2(1)=-width/2;
	J2(2)=1;
	J2(3)=width/2;

	//note this is the inverse of above.
	J=Eigen::MatrixXd(2,2);
	J(0)=.5;
	J(1)=.5;
	J(2)=-1/width;
	J(3)=1/width;

	kl=kr=.1;

}

DifferentialKinematics::~DifferentialKinematics() {
	//this is a goddamn defined destructor.
}

/**
 * Returns [left speed; right speed] required to realize the given velocities. Pass in as: [linear velocity (along x); angular (about Z)]
 */
Eigen::MatrixXd DifferentialKinematics::getWheelSpeeds(Eigen::MatrixXd& velocities){
	return J2*velocities;
}

/**
 * Returns [linear velocity (along x); angular (about Z)] given [left speed; right speed]
 */
Eigen::MatrixXd DifferentialKinematics::getVelocities(Eigen::MatrixXd& wheelspeeds){
	return J*wheelspeeds;
}

/**
 * Returns [left;right] velocities given desired linear and angular velocities.
 */
Eigen::MatrixXd DifferentialKinematics::getWheelSpeeds(float linear, float angular){
	Eigen::MatrixXd ret = Eigen::MatrixXd(2,1);
	ret(0)=linear;
	ret(1)=angular;
	return J2*ret;
}

/**
 * returns the transformation between frames, after the robot has traveled with odometer readings=[left;right].
 */
void inline DifferentialKinematics::getTransformationFromDistances(Eigen::MatrixXd& dists, Eigen::MatrixXd& out_T){
	getTransformationFromDistances(dists(0),dists(1),out_T);
}
void DifferentialKinematics::ekfMeasurement(Eigen::MatrixXd&X,Eigen::MatrixXd&Q,Eigen::MatrixXd&Z,Eigen::MatrixXd&R){
	Eigen::MatrixXd I=Eigen::MatrixXd::Identity(3,3);
	Eigen::MatrixXd ZP = Eigen::MatrixXd(Z);
	Eigen::MatrixXd XP = Eigen::MatrixXd(X);
	Eigen::MatrixXd z = Z-X;
	Eigen::MatrixXd S =Q+R;
	Eigen::MatrixXd Sp=S;
	//invert(S,Sp);
	Eigen::MatrixXd K=Q*S.inverse();
	//dumpMatrix(K);
	X = X + K*(z);
	Q = (I-K)*Q;
}
/**
 * Both are in/out
 */
void DifferentialKinematics::ekfPredict(float left, float right, Eigen::MatrixXd&pose, Eigen::MatrixXd&cv){
	float hth=pose(2)+getTheta(left,right)/2;
	float dd = std::fabs(getVel(left,right));
	updatePose(left,right,pose);

	Eigen::MatrixXd V=Eigen::MatrixXd::Zero(3,2);
	V(0,0)=(cos(hth)+(dd/width)*sin(hth))/2;
	V(0,1)=(cos(hth)-(dd/width)*sin(hth))/2;
	V(1,0)=(sin(hth)-(dd/width)*cos(hth))/2;
	V(1,1)=(sin(hth)+(dd/width)*cos(hth))/2;
	V(2,0)=(-1/width);
	V(2,1)=(1/width);

	Eigen::MatrixXd G;
	G=Eigen::MatrixXd::Identity(3,3);
	G(0,2)=-dd*sin(hth);
	G(1,2)=cos(hth)*dd;
//	G=V.clone();

	Eigen::MatrixXd M=Eigen::MatrixXd::Zero(2,2);
	M(0,0)=left<0? -1*left*kl:left*kl;
	M(1,1)=right<0? -1*right*kr:right*kr;

	Eigen::MatrixXd R=V*M*V.transpose();
	Eigen::MatrixXd r2 = G*cv*G.transpose()+R;
	cv = r2;
	return;
}
/**
 * outputs the transformation between frames, after the robot has traveled with odometer readings=[left;right].
 */
void DifferentialKinematics::getTransformationFromDistances(float left, float right, Eigen::MatrixXd& out_T){
	float dd = (left+right)/2;
	float dth = (right-left)/width;
	out_T=Eigen::MatrixXd::Identity(3,3);
	if (dth<.000001 && dth> -.000001){
		out_T(0,2)=dd;
	} else {
		float s = sin(dth/2);
		float c = cos(dth/2);
		float c2 = cos(dth);
		float s2 = sin(dth);
		float radius = dd/dth;
		float disp = 2*radius*s;
		out_T(0,2)=disp*c;
		out_T(1,2)=disp*s;
		out_T(0,0)=c2;
		out_T(0,1)=-s2;
		out_T(1,0)=s2;
		out_T(1,1)=c2;
	}
}
float DifferentialKinematics::getTheta(float left, float right){
	float dth = (right-left)/width;
	return dth;
}
float DifferentialKinematics::getVel(float left, float right){
	float dd = (left+right)/2;
	return dd;
}

void DifferentialKinematics::updatePose(float left, float right, Eigen::MatrixXd&out_Pose){
	float dth = getTheta(left,right);
	float v = getVel(left,right);
	float dx,dy;
	float px,py,pth;
	px = out_Pose(0);
	py = out_Pose(1);
	pth = out_Pose(2);
	dx = cos(pth+dth/2)*v;
	dy = sin(pth+dth/2)*v;
	out_Pose(0)=dx+px;
	out_Pose(1)=dy+py;
	out_Pose(2)=dth+pth;
	while (out_Pose(2)>M_PI){
		out_Pose(2)-=2*M_PI;
	}
	while (out_Pose(2)<-M_PI){
		out_Pose(2)+=2*M_PI;
	}
}
