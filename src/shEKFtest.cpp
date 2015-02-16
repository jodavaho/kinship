/*
 * shEKFtest.cpp
 *
 *  Created on: May 28, 2011
 *      Author: joshua
 */

#include <Kinematics/SingleHullKinematics.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

using namespace RSN;
using namespace cv;


SingleHullKinematics sh;

struct CurConst{
	float heading;
	float str;
	float heading_cov;
	float str_cov;
}cc;

void dump(Mat_<float>pose){
	printf("X:[");
	for (int i=0;i<pose.rows;i++){
		printf(" %0.3f ",pose(i));
	}
	printf("]");
}
void dump(Mat_<float> pose, Mat_<float> cv){
	dump(pose);
	printf(" \u03C3:{");
	for (int i =0;i<cv.rows;i++){
		printf("(%0.3f)",cv(i,i));
	}
	printf("}");
}
void pp(SingleHullState X,float th,float an,float t){
	printf("-");dump(X.state,X.cov);
	Mat_<float> acc = Mat::zeros(3,1,CV_32F);
	Mat_<float> oldCV = X.cov.clone();
	Mat_<float> cv = X.cov;
	sh.getAccVectors(th,an,X.state,acc);
	sh.ekfPredict(th,an,t,X);
//	printf("-- ddPr/ddt: %0.3f,%0.3f (%0.3f) ",acc(0),acc(1),acc(2));
	printf("\n");
	printf("+");
	dump(X.state,X.cov);
	printf("\n");
	for (int i=0;i<cv.rows;i++){
		if (oldCV(i,i)>cv(i,i)){
					printf("\n!!! %d !!! %0.3f --> %0.3f \n",i,oldCV(i,i),cv(i,i));

				}
//		assert(oldCV(i,i)<=cv(i,i));
	}
}
void pp(Mat_<float> pose,Mat_<float> cv, float th, float an, float t){
	Mat_<float> acc = Mat::zeros(3,1,CV_32F);
	Mat_<float> oldCV = cv.clone();
	printf("-");dump(pose,cv);
	sh.getAccVectors(th,an,pose,acc);
	sh.ekfPredict(th,an,t,pose,cv);
//	printf("-- ddPr/ddt: %0.3f,%0.3f (%0.3f) ",acc(0),acc(1),acc(2));
	printf("\n");
	printf("+");
	dump(pose,cv);
	printf("\n");
	for (int i=0;i<cv.rows;i++){
		if (oldCV(i,i)>cv(i,i)){
			printf("!!! %d !!!\n",i);
		}
		assert(oldCV(i,i)<=cv(i,i));
	}
}

void doC(Mat_<float> x){
	printf("x:");
	dump(x);
	float cd = ((float)rand()/RAND_MAX-.5)*cc.heading_cov + cc.heading;
	float cs = ((float)rand()/RAND_MAX-.5)*cc.str_cov +cc.str;
	printf("+ c^ %0.3f, |c| %0.3f ",cd,cs);

	x(0)+=cos(cd)*cs;
	x(1)+=sin(cd)*cs;

	dump(x);
	printf("\n");
}

void getM(Mat_<float> x, Mat_<float> z)
{
	z=Mat::zeros(6,1,CV_32F);
	z(0)=0+((float)rand()/RAND_MAX-.5)*5;
	z(1)=0+((float)rand()/RAND_MAX-.5)*5;
	z(2)=0+((float)rand()/RAND_MAX-.5)*1;
	z(3)=x(3);//0+((float)rand()/RAND_MAX-.5)*2;
	z(4)=x(4);//0+((float)rand()/RAND_MAX-.5)*2;
	z(5)=x(5);//0+((float)rand()/RAND_MAX-.5)*0;
	z=z+x;
}
void pmc(SingleHullState X,float d,float s){

	d = d+(((float)rand())/RAND_MAX-.5)*s;
	float dr = compassToXAngleRad(d);
	float dd = diff(dr,X.state.at<float>(2),-M_PI,M_PI);
	printf("compass z: %0.3f (+/-%0.3f) deg (%0.3f rad)-- diff: %0.3f\n",d,s,dr,dd);
	sh.compassMeasurement(d,s,X);
	printf("x|z:");
	dump(X.state,X.cov);
	printf("\n");

}
void pmgps(SingleHullState X,float deg, float sdeg){
	//generate false GPS measure
	GPS_Measurement Z;
	Z.cov = Mat::eye(2,2,CV_32F)*5;
	Z.position = X.state.rowRange(0,2).clone();

	//with noise
	Mat_<float> n = Mat::zeros(2,1,CV_32F);
	n(0)=((float)rand()/RAND_MAX-.5)*5;
	n(1)=((float)rand()/RAND_MAX-.5)*5;
	Z.position = Z.position + n;

	//do track
	Z.track=deg+((float)rand()/RAND_MAX-.5)*sdeg;
	Z.err_track = sdeg+20;

	double dr = compassToXAngleRad(Z.track);
	printf("GPS Z: \u0398:%0.3f, (+/-%0.3f) deg (%0.3f rad)--",Z.track,Z.err_track,dr);
	dump(Z.position,Z.cov);
	printf("\n");
	sh.localGPSMeasure(Z,X);
	printf("x|z:");
	dump(X.state,X.cov);
	printf("\n");

}
void pm(Mat_<float> x, Mat_<float> cx,Mat_<float> z,Mat_<float> cz){

	printf("z:");
	dump(z,cz);
	printf("\n");
	Mat_<float> oldCX = cx.clone();

	Mat I = Mat::eye(6,6,CV_32F);

	sh.ekfMeasurement(x,x,I,cx,z,cz);
	printf("x|z:");
	dump(x,cx);
	printf("\n");
//
//	for (int i=0;i<oldCX.rows;i++){
//		if (i!=2&&i!=5)
//		{
//		bool ok;
//		ok = oldCX(i,i)<=cx(i,i) && cx(i,i)<= cz(i,i);
//		ok = ok || (cx(i,i)<=oldCX(i,i) && cz(i,i)<= cx(i,i));
//		assert(ok);
//		}
//	}

}
int main(int argc, char** argv){

	cc.heading=M_PI;
	cc.str=.1;

	srand(time(NULL));
	printf("starting test\n");

	Mat_<float> cx = Mat::eye(6,6,CV_32F)*2;
	Mat_<float> cz = Mat::eye(6,6,CV_32F)*5;
	Mat_<float> z= Mat::zeros(6,1,CV_32F);
	Mat_<float> x = Mat::zeros(6,1,CV_32F);
	x(2)=-M_PI/4;
	int COMP_HEAD = 135;


	SingleHullConstants cc = sh.getConstants();

	cc.Kr=10;
	cc.Kx=10;
	cc.Ky=20;

	cc.Mx=10;
	cc.My=10;
	cc.Ir=3;

	cc.Kprop=0.0;
	cc.dp=1.0;

	cc.s_an=.1;
	cc.s_th=.25;
	cc.mn_th=.01;

	sh.setConstants(cc);

	SingleHullState s;
	s.state = x;
	s.cov = cx;

	int STEPS = 5;

	printf("\t\t waiting...\n");
		for (int j=0;j<STEPS;j++){
			for (int i =0;i<3;i++){
				pmc(s,COMP_HEAD,10);
				for (int i =0;i<5;i++){
					pp(s,0,0,.1);
				}
			}
			pmgps(s,COMP_HEAD,10);
		}

	printf("\t\t ahead slow!\n");
	for (int j=0;j<STEPS;j++){
		for (int i =0;i<3;i++){
			pmc(s,COMP_HEAD,10);
			for (int i =0;i<5;i++){
				pp(s,10,0,.1);
			}
		}
		pmgps(s,COMP_HEAD,10);
	}
	printf("\t\t Right Rudder!\n");
	for (int j=0;j<STEPS;j++){
		for (int i =0;i<3;i++){
			pmc(s,COMP_HEAD,10);
			for (int i =0;i<5;i++){
				pp(s,10,.2,.1);
			}
		}
		pmgps(s,COMP_HEAD,10);
	}
	printf("\t\t rudder midships\n");
	for (int j=0;j<STEPS;j++){
		for (int i =0;i<3;i++){
			pmc(s,COMP_HEAD,10);
			for (int i =0;i<5;i++){
				pp(s,10,0,.1);
			}
		}
		pmgps(s,COMP_HEAD,10);
	}


	return 0;
}
