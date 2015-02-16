#include <Kinematics/SingleHullKinematics.h>

using namespace cv;
using namespace RSN;

//#define __DEBUG

SingleHullKinematics::SingleHullKinematics(){
	cc.Kprop=0.0;
}
SingleHullKinematics::SingleHullKinematics(SingleHullConstants cc){
	this->cc=cc;
}
SingleHullKinematics::~SingleHullKinematics(){

}
void SingleHullKinematics::ekfPredictSimple(float thrust, float angle, float t, Mat&pose, Mat& cv){
	Mat_<float> p = pose.rowRange(0,3);
	Mat_<float> dp = pose.rowRange(3,6);

	float thd = p(2)+dp(2)*t/2;
	float cd = cos(thd);
	float sd = sin(thd);
	float ca = cos(angle);
	float sa = sin(angle);

	updatePose(thrust,angle,t,pose);

	Mat_<float> V=Mat::zeros(6,2,CV_32F);
	V(3,0)=ca/cc.Mx; V(3,1)=-sa*thrust/cc.Mx;// - 2*sa*cc.Kprop;
	V(4,0)=sa/cc.My; V(4,1)=ca*thrust/cc.My;
	V(5,0)=cc.dp*sa/cc.Ir; V(5,1)=cc.dp*ca*thrust/cc.Ir;

	Mat_<float> G=Mat::eye(6,6,CV_32F);
	G(0,2)=	-dp(0)*sd*t - dp(1)*cd*t;
	G(0,3)= cd*t;
	G(1,2)= dp(0)*cd*t - dp(1)*sd*t;
	G(1,3)= sd*t;

//	G(0,4)= sd*t;
//	G(0,5)=-t*t*.5*(sd*dp(0)+cd*dp(1));
//	G(1,4)= cd*t;
//	G(1,5)=t*t*.5*(cd*dp(0)-sd*dp(1));

//	G(2,5)=t;

	G(3,3)=-cc.Kx*t;
	G(4,4)=-cc.Ky*t;
	G(5,5)=-cc.Kr*t;

	Mat_<float> M=Mat::zeros(2,2,CV_32F);
	M(0,0)= cc.s_th*(thrust!=0? cc.mn_th:0);
	M(1,1)= cc.s_an;

	Mat R=V*M*V.t();
	Mat r2 = G*cv*G.t()+R+Mat::eye(6,6,CV_32F)*.3; //salt it a little.
	r2.copyTo(cv);
	return;
}
void SingleHullKinematics::ekfPredictActual(float thrust, float angle, float t, Mat&pose, Mat&cv){
//	printf("/?4 %0.3f\n",pow);
	Mat_<float> p = pose.rowRange(0,3);
	Mat_<float> dp = pose.rowRange(3,6);

	//orientation of the vector that points to the new location
	float thAct = p(2)+dp(2)*t/2+atan2(dp(1),dp(0));

	float cAct = cos(thAct);
	float sAct = sin(thAct);

	float xp = dp(0);
	float yp = dp(1);

	float ca = cos(angle);
	float sa = sin(angle);

	updatePose(thrust,angle,t,pose);

//	float thrust = pow;
//	thrust = (std::abs(pow)<.01?0:pow/std::abs(pow))*t;
	float absVel = norm(dp.rowRange(0,2));
//	absVel = std::abs(absVel)>.01?thrust:0;
//	absVel = std::abs(absVel)>.01?absVel/std::abs(absVel):0;
	float SC = absVel==0? 0:t/absVel;


#ifdef __DEBUG
	printf("EKF Predict:\n");
	dumpMatrix(pose);
#endif

	Mat_<float> V=Mat::zeros(6,2,CV_32F);
	V(3,0)=ca/cc.Mx; V(3,1)=-sa*.1/cc.Mx;// - 2*sa*cc.Kprop;
	V(4,0)=sa/cc.My; V(4,1)=ca*.1/cc.My;
	V(5,0)=cc.dp*sa/cc.Ir; V(5,1)=cc.dp*ca*.1/cc.Ir;

	Mat_<float> G=Mat::eye(6,6,CV_32F);
	//dfx/dth
	G(0,2)=	-t*sAct*absVel;
	//dfx/dxp
	G(0,3)= SC*(xp*cAct+yp*sAct);
	//dfx/dyp
	G(0,4)= SC*(yp*cAct-xp*sAct);
	//dfx/domega
	G(0,5)= -(t*t)/2 * absVel * sAct;

	//dfy/dth
	G(1,2)= cAct*absVel*t;
	//dfy/xp
	G(1,3)= SC*(xp*sAct-yp*cAct);
	//dfy/yp
	G(1,4)= SC*(xp*cAct+yp*sAct);
	//dfy/domega
	G(1,5)=(t*t)/2 * absVel * cAct;

	//df(theta)/domega
	G(2,5)=t;

	//velocity jacobians:
	G(3,3)=1-cc.Kx*t;
	G(4,4)=1-cc.Ky*t;
	G(5,5)=1-cc.Kr*t;

	//hmmm...
	G(3,3)=0;
	G(4,4)=0;
	G(5,5)=0;

	Mat_<float> M=Mat::zeros(2,2,CV_32F);
	M(0,0)= std::pow(thrust,2)*cc.s_th;//cc.s_th*(std::abs(thrust)==0?cc.mn_th:thrust);
	M(1,1)= cc.s_an;

	Mat R=V*M*V.t();

	Mat salt = Mat::eye(6,6,CV_32F)*t;
	Mat r2 = G*cv*G.t()+R+salt;
#ifdef __DEBUG
	printf("Matricies: (G,cv,R,salt, and result)\n");
	dumpMatrix(G);
	dumpMatrix(cv);
	dumpMatrix(R);
	dumpMatrix(r2);
#endif
	r2.copyTo(cv);
	return;
}
//Public passthrough
void SingleHullKinematics::ekfPredict(float thrust, float angle, float t, SingleHullState& state){
	ekfPredict(thrust,angle,t,state.state,state.cov);
}
//Public
void SingleHullKinematics::ekfPredict(float thrust, float angle, float t, Mat&pose, Mat& cv){
	ekfPredictActual(thrust,angle,t,pose,cv);
}
void SingleHullKinematics::compassMeasurement(float degrees,float sig_degrees, SingleHullState& state){

	//compass measures heading directly, and ignores drift.
	Mat_<float> X = state.state;
	Mat_<float> Q = state.cov;
	float mx = X(2);

	mx = normalizeAngle(mx);

	float mz = compassToXAngleRad(degrees);
	float cx = Q(2,2);
	float cz = std::abs(degToRad(sig_degrees));

	//carefully measure residualation.
	float residual = normalizeAngle(diff(mz,mx,-M_PI,M_PI));

#ifdef __DEBUG
	printf("Compass: X:%0.3f, Z:%0.3f, residual:%0.3f\n",mx,mz,residual);
	printf("Compass -- X+res: %0.3f, Z:%0.3f\n",normalizeAngle(mx+residual),normalizeAngle(mz));
	assert(std::abs(normalizeAngle(mx+residual))-std::abs(normalizeAngle(mz))<.001);
#endif

	float k = cx/(cx+cz);
	X(2) = mx + k*(residual);
	X(2) = normalizeAngle(X(2));
	Q(2,2) = (1-k)*(cx);
#ifdef __DEBUG
	printf("Compass: (Z,cov(Z),hx,z,x_final)\n");
	printf("\tZ:%0.3f (cz:%0.3f) --> z_rad:%0.3f\n",degrees,cz,mz);
	printf("\tX:%0.3f (cx:%0.3f) --> z-x:%0.3f\n",mx,cx,residual);
	printf("\t =%0.3f (+/- %0.3f)\n",X(2),Q(2,2));
#endif

}
/**
 * This only requires the struct, you do not have to pre process the GPS packet at all.
 */
void SingleHullKinematics::localGPSMeasure(GPS_Measurement Z, SingleHullState&X){

#ifdef __DEBUG
	printf("GPS Measurement: (Z,cov(Z)\n");
	dumpMatrix(Z.position);
	dumpMatrix(Z.cov);
	printf("GPS speed: %0.3f,%0.3f\n",Z.speed,Z.err_speed);
	printf("GPS track: %0.3f,%0.3f\n",Z.track,Z.err_track);
#endif
	//operate only on position and heading information. The GPS cannot provide information
	//about any other part of the state, not even velocity vectors, sadly.
	Mat_<float> x = X.state.rowRange(0,3);
	Mat_<float> q = Mat(X.cov,Range(0,3),Range(0,3));
	Mat_<float> QQ = X.cov;

	//calculate h(X)
	Mat_<float> hX = Mat::zeros(x.rows,1,X.state.type());
	hX(0) = x(0);
	hX(1) = x(1);
	hX(2) = normalizeAngle(x(2) + atan2(x(4),x(3))); //gps measures drift + heading

	//prepare measurement vector
	Mat_<float> z = Mat::zeros(3,1,CV_32F);
	z(0) = Z.position.at<float>(0);
	z(1) = Z.position.at<float>(1);
	z(2) = compassToXAngleRad(Z.track);

	//measurement covariance:
	Mat_<float> r = Mat::zeros(3,3,CV_32F);
	r(2,2) = std::abs(degToRad(Z.err_track));
	Mat_<float> rr = r;

	Z.cov.copyTo(rr); //copy into upper 2x2 of r.
	r(0,0)=Z.cov.at<float>(0,0);
	r(0,1)=Z.cov.at<float>(0,1);
	r(1,0)=Z.cov.at<float>(1,0);
	r(1,1)=Z.cov.at<float>(1,1);

	//measurement jacobian
	Mat_<float> H = Mat::eye(3,3,CV_32F);

	//should be H(2,3) and H(2,4) terms.. but they are smallish. order of 1/x^2
#ifdef __DEBUG
	printf("GPS premeasure: (x,hx,H,q,z,r)\n");
	dumpMatrix(x);
	dumpMatrix(hX);
	dumpMatrix(H);
	dumpMatrix(q);
	dumpMatrix(z);
	dumpMatrix(r);
#endif
	ekfMeasurement(x,hX,H,q,z,r);
#ifdef __DEBUG
	printf("GPS post measure: (x,hx,H,q,z,r)\n");
	dumpMatrix(x);
	dumpMatrix(hX);
	dumpMatrix(H);
	dumpMatrix(q);
	dumpMatrix(z);
	dumpMatrix(r);
#endif
	X.state.at<float>(0)=x(0);
	X.state.at<float>(1)=x(1);
	X.state.at<float>(2)=x(2);

	QQ(0,0)=q(0,0);
	QQ(0,1)=q(0,1);
	QQ(0,2)=q(0,2);
	QQ(1,0)=q(1,0);
	QQ(1,1)=q(1,1);
	QQ(1,2)=q(1,2);
	QQ(2,0)=q(2,0);
	QQ(2,1)=q(2,1);
	QQ(2,2)=q(2,2);
}

void SingleHullKinematics::ekfMeasurement(Mat&X, Mat&hOfX, Mat H, Mat&Q, Mat&Z, Mat&R){
	Mat_<float>XP = X;

	Mat I=Mat::eye(hOfX.rows,hOfX.rows,hOfX.type());
	assert(hOfX.size==Z.size);
	assert(Q.size==R.size);
	assert(H.size==Q.size);

	Mat_<float> z = Z-hOfX;
	//all ok, except:
	z(2) = normalizeAngle(diff(Z.at<float>(2),hOfX.at<float>(2),-M_PI,M_PI));
	//...because we must be careful of angles.

#ifdef __DEBUG
	printf("residual:\n");
	dumpMatrix(z);
#endif

	Mat S =H*Q*H.t()+R;
	Mat Sp=S.clone();
	invert(S,Sp);
	Mat K=Q*H.t()*Sp;
#ifdef __DEBUG
	printf("ekf Measure: Z,Q,K,K*(Z-hX)\n");
	dumpMatrix(Z);
	dumpMatrix(Q);
	dumpMatrix(K);
	dumpMatrix(K*z);
#endif
	XP = XP + K*(z);
	XP(2) = normalizeAngle(XP(2));
	Q = (I-K*H)*Q;
}

void SingleHullKinematics::setConstants(SingleHullConstants c){
	cc=c;
}
SingleHullConstants SingleHullKinematics::getConstants(){
	return cc;
}

void SingleHullKinematics::getAccVectors(float thrust, float angle, Mat pose, Mat out_acc){

	Mat_<float> out = out_acc;
	Mat_<float> p = pose;
	Mat_<float> K = Mat::zeros(3,6,CV_32F);
	Mat_<float> U = Mat::zeros(2,1,CV_32F);
	U(0)=thrust;
	U(1)=angle;

	K(0,3)=-cc.Kx*std::abs(p(3));
	K(1,4)=-cc.Ky*std::abs(p(4));
	K(2,5)=-cc.Kr*std::abs(p(5));

	Mat_<float> M = Mat::zeros(3,2,CV_32F);

	M(0,0)=cos(angle)/cc.Mx; M(0,1) = 0.0;
	M(1,0)=sin(angle)/cc.My; M(1,1) = 0.0;
	M(2,0)=-sin(angle)*cc.dp/cc.Ir; M(2,1) = 0.0;

	Mat_<float> frict = K*p;
	Mat_<float> acc = M*U;

	out_acc = frict + acc;
#ifdef __DEBUG
	printf("Acc Terms (K,p,M,U,acc)::\n");
	dumpMatrix(K);
	dumpMatrix(p);
	dumpMatrix(M);
	dumpMatrix(U);
	dumpMatrix(out_acc);
#endif

}
void enforceConstraints(Mat pose){
	Mat_<float> t= pose;

	assert(t(3)<7.0 && t(3)>-7.0);
	assert(t(4)<7.0 && t(4)>-7.0);
	assert(t(5)<5.0 && t(5)>-5.0);
	assert(t(2)>=-M_PI&&t(2)<=M_PI);

}

// x+ = g(x-, u-, dt);
void SingleHullKinematics::updatePose(float thrust, float angle, float time, Mat&pose){
//	assert(thrust<=1.0 && thrust >= -1.0);
//	assert(angle>=-M_PI && angle<=M_PI);
	assert(time<10.0);
	//pose == position
	Mat_<float> p = pose.rowRange(0,3);
	//dp/dt * t
	Mat_<float> dptimest = Mat(3,1,CV_32F);
	//dp/dt == velocity
	Mat_<float> dpdt = pose.rowRange(3,6);
	//ddp/ddt == acceleration
	Mat_<float> acc = Mat::zeros(3,1,CV_32F);

	float thin = atan2(dpdt(1),dpdt(0));
	float thd = p(2)+dpdt(2)*time/2+thin;

	//direct path length poses == ||dP/dt|| * t
	float dd = norm(Mat(dpdt,Range(0,2))) * time;
	//chord length
	float chl=0.0;
	//angle change
	float dang = dpdt(2)*time;

	if (dang==0){
		chl=dd;
	}else if (dang<=M_PI && dang>=-M_PI){
		//2 * sin(angle/2) * (Rad = arc dist / arc angle) = chord length.
		chl = 2*sin(dang/2)*(dd/dang);
	}

	getAccVectors(thrust,angle,pose,acc);
//	dumpMatrix(ddpddt);

	dptimest(0) = cos(thd)*chl;
	dptimest(1) = sin(thd)*chl;
	dptimest(2) = dpdt(2)*time;

//	dpt(0)= cos(thd)*dp(0)*time-sin(thd)*dp(1)*time;
//	dpt(1)= sin(thd)*dp(0)*time+cos(thd)*dp(1)*time;
//	dpt(2)=dp(2)*time;

	//update position in global frame
	p=p+dptimest;

	p(2) = normalizeAngle(p(2));

	//update velocities in local frame.
	dpdt = dpdt + acc*time;

//	enforceConstraints(pose);
//	warnConstraints(pose);
}

