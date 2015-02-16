/*
 * angleTest.cpp
 *
 *  Created on: May 29, 2011
 *      Author: joshua
 */
#include <math.h>
#include <helpers/math.h>
#include <stdio.h>
#define _USE_MATH_DEFINES

using namespace RSN;

double test(double Z, double hX){
	double R=diff(Z,hX,-M_PI,M_PI);
	R = normalizeAngle(R);
	printf("\t %0.3f - (%0.3f) = %0.3f <=> %0.3f + %0.3f = %0.3f\n",Z,hX,R,hX,R,normalizeAngle(R+hX));
	return R;
}
double ctest(double r){
	double d = compassToXAngleRad(r);
	printf("%0.3f-->%0.3f\n",r,d);
}
int main(){
	ctest(0.0);
	ctest(90.0);
	ctest(180.0);
	ctest(270.0);
	ctest(269.0);
	ctest(271.0);
	ctest(359.0);
	ctest(1.0);
	test(0,0);
	test(-3.108,2.950);
	test(0,M_PI);
	test(0,2*M_PI);
	test(M_PI/4,-M_PI/4);
	test(.1,-.001);
	test(-M_PI/2-.001,-M_PI/2+.001);
	test(-M_PI/2+.001,-M_PI/2-.001);
	return 0;
}
