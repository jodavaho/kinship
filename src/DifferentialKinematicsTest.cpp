#include <opencv2/opencv.hpp>
#include <Kinematics/DifferentialKinematics.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

using namespace cv;
using namespace RSN;

void dumpMatrix(Mat m){
	printf("\n");
	for (int j =0;j<m.rows;j++){
		printf("\t");
		for (int i=0;i<m.cols;i++){
			printf(" %0.3f ",m.at<float>(j,i));
		}
		printf("\n");
	}
	printf("\n");
}

int main(int argc, char** argv){

	srand(time(NULL));
	DifferentialKinematics kins(.3);
	kins.kl=1.0;
	kins.kr=1.0;

	for (int i =0;i<1;i++){
		Mat X = -Mat::ones(3,1,CV_32F);
		Mat Q = Mat::eye(3,3,CV_32F);
		Mat z = -(Mat::ones(3,1,CV_32F)+1);
		Mat R = Mat::eye(3,3,CV_32F)*.4;
		printf("x: %0.3f (%0.3f) y: %0.3f (%0.3f) th: %0.3f (%0.3f)\n",
			X.at<float>(0),Q.at<float>(0,0),
			X.at<float>(1),Q.at<float>(1,1),
			X.at<float>(2),Q.at<float>(2,2));
		printf("z: %0.3f (%0.3f) y: %0.3f (%0.3f) th: %0.3f (%0.3f)\n",
								z.at<float>(0),R.at<float>(0,0),
								z.at<float>(1),R.at<float>(1,1),
								z.at<float>(2),R.at<float>(2,2));
		kins.ekfMeasurement(X,Q,z,R);
		printf("x: %0.3f (%0.3f) y: %0.3f (%0.3f) th: %0.3f (%0.3f)\n",
					X.at<float>(0),Q.at<float>(0,0),
					X.at<float>(1),Q.at<float>(1,1),
					X.at<float>(2),Q.at<float>(2,2));
	}

	Mat_<float> X = Mat::zeros(3,1,CV_32F);
	X(2)=0;
	X(1)=0;
	X(0)=0;
	Mat Q = Mat::eye(3,3,CV_32F)*50;
	kins.kl=.5;
	kins.kr=.5;
	//pose.at<float>(2)=M_PI/2;
	Mat H = Mat::eye(3,3,CV_32F);
	Mat R = Mat::eye(3,3,CV_32F)*20;
	R.at<float>(2,2)=1;
	for (int i =1;i<100;i++){

		int r = 10;
		float c1 = 1.0/r;
		float c2 = 1.0/r;
		for (int j=0;j<r;j++){
			printf("x: %0.3f (%0.3f) y: %0.3f (%0.3f) th: %0.3f (%0.3f)\n",
				X.at<float>(0),Q.at<float>(0,0),
				X.at<float>(1),Q.at<float>(1,1),
				X.at<float>(2),Q.at<float>(2,2));
			printf("Control %d: %0.3f, %0.3f\n",j,c1,c2);
			kins.ekfPredict(c1,c2,X,Q);
			printf("x: %0.3f (%0.3f) y: %0.3f (%0.3f) th: %0.3f (%0.3f)\n",
				X.at<float>(0),Q.at<float>(0,0),
				X.at<float>(1),Q.at<float>(1,1),
				X.at<float>(2),Q.at<float>(2,2));
		}

		Mat Z = Mat::zeros(3,1,CV_32F);
		Mat_<float> z = Mat_<float>(Z);
//		z(2)=X(2)+((float)rand()/RAND_MAX-.5)*.2;
//		z(1)=X(1)+((float)rand()/RAND_MAX-.5)*5;
//		z(0)=X(0)+((float)rand()/RAND_MAX-.5)*5;
		z(2)=0+((float)rand()/RAND_MAX-.5)*.2;
		z(1)=0+((float)rand()/RAND_MAX-.5)*5;
		z(0)=i+((float)rand()/RAND_MAX-.5)*5;
		printf("z: %0.3f (%0.3f) y: %0.3f (%0.3f) th: %0.3f (%0.3f)\n",
						z.at<float>(0),R.at<float>(0,0),
						z.at<float>(1),R.at<float>(1,1),
						z.at<float>(2),R.at<float>(2,2));

		kins.ekfMeasurement(X,Q,z,R);
		printf("x: %0.3f (%0.3f) y: %0.3f (%0.3f) th: %0.3f (%0.3f)\n",
				X.at<float>(0),Q.at<float>(0,0),
				X.at<float>(1),Q.at<float>(1,1),
				X.at<float>(2),Q.at<float>(2,2));
		printf("\n");
	}

}
