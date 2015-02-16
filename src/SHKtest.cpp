/*
 * SHKtest.cpp
 *
 *  Created on: May 26, 2011
 *      Author: joshua
 */
#include <Kinematics/SingleHullKinematics.h>
#include <stdio.h>


using namespace cv;
using namespace RSN;

SingleHullKinematics sh;

void pp(Mat_<float> pose,float th, float an, float t){
	Mat_<float> acc = Mat::zeros(3,1,CV_32F);
	printf("-- Pg: %0.3f,%0.3f (%0.3f)",pose(0),pose(1),pose(2));
	printf("-- dPr/dt: %0.3f, %0.3f (%0.3f)",pose(3),pose(4),pose(5));
	sh.getAccVectors(th,an,pose,acc);
	sh.updatePose(th,an,t,pose);
	printf("-- ddPr/ddt: %0.3f,%0.3f (%0.3f)",acc(0),acc(1),acc(2));
	printf("\n");
}
int main(int argc, char** argv){

	printf("starting test\n");

	Mat_<float> vvv = Mat(4,1,CV_32F);
	for (int i =0;i<4;i++){
		vvv(i)=i;
	}
	Mat_<float>t = vvv.rowRange(0,1);
	printf("----%d----\n",t.rows);

	SingleHullConstants cc = sh.getConstants();

	cc.Kr=3;
	cc.Kx=1;
	cc.Ky=5;

	cc.Mx=10;
	cc.My=10;
	cc.Ir=3;

	cc.Kprop=0.0;
	cc.dp=1.0;
	sh.setConstants(cc);

	printf("starting pose\n");
	Mat_<float> pose = Mat::zeros(6,1,CV_32F);

	int STEPS = 20;

	printf("\t\t ahead slow!\n");
	for (int i =0;i<STEPS;i++){
		pp(pose,10,0,.1);
	}

	printf("\t\t right rudder!\n");
	for (int i =0;i<2*STEPS;i++){
		pp(pose,10,.5,.1);
	}
	printf("\t\trudder amidships!\n");
		for (int i =0;i<3*STEPS;i++){
			pp(pose,10,0,.1);
		}

	printf("\t\tleft rudder!\n");
	for (int i =0;i<STEPS;i++){
		pp(pose,10,-.5,.1);
	}
	printf("\t\trudder amidships!\n");
	for (int i =0;i<STEPS;i++){
		pp(pose,10,0,.1);
	}
	printf("\t\tengines full stop!\n");
	for (int i =0;i<STEPS;i++){
		pp(pose,0,0,.1);
	}		/**/

	return 0;
}
