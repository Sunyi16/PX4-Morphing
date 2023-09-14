/**
 * @file ladrc.c
 *
 * Implementation of generic LADRC controller.
 *
 * @author sunyi
 */

#include"ladrc.h"

void LADRC_init(LADRC_param *LADRC_param_Pitch , LADRC_param *LADRC_param_Roll , LADRC_param *LADRC_param_Yaw)
{
	/*pitch通道调节参数*/
	LADRC_param_Pitch->h=200;	//定时时间及时间步长
	LADRC_param_Pitch->r=1000;	//跟踪速度参数
	LADRC_param_Pitch->w0=30;	//观测器带宽
	LADRC_param_Pitch->wc=3;	//状态误差反馈率带宽
	LADRC_param_Pitch->b0=1;
	LADRC_param_Pitch->Kp=0.01;
	LADRC_param_Pitch->Kd=100000;


	/*roll通道调节参数*/
	LADRC_param_Roll->h=200;	//定时时间及时间步长
	LADRC_param_Roll->r=1000;	//跟踪速度参数
	LADRC_param_Roll->w0=600;	//观测器带宽
	LADRC_param_Roll->wc=3;	//状态误差反馈率带宽
	LADRC_param_Roll->b0=1;
	LADRC_param_Roll->Kp=0.01;
	LADRC_param_Roll->Kd=100000;

	/*yaw通道调节参数*/
	LADRC_param_Yaw->h=200;	//定时时间及时间步长
	LADRC_param_Yaw->r=1000;	//跟踪速度参数
	LADRC_param_Yaw->w0=600;	//观测器带宽
	LADRC_param_Yaw->wc=15;	//状态误差反馈率带宽
	LADRC_param_Yaw->b0=1;
	LADRC_param_Yaw->Kp=0.01;
	LADRC_param_Yaw->Kd=100000;


}

void LADRC_TD(LADRC_param LADRC_param_One , float Expect)//跟踪微分部分
{

	float fh= -LADRC_param_One.r*LADRC_param_One.r*(LADRC_param_One.v1-Expect)-2*LADRC_param_One.r*LADRC_param_One.v2;
	LADRC_param_One.v1+=LADRC_param_One.v2*LADRC_param_One.h;
	LADRC_param_One.v2+=fh*LADRC_param_One.h;
}

void LADRC_ESO(LADRC_param LADRC_param_One , float FeedBack)//状态观测器
{
	float Beita_01=3*LADRC_param_One.w0;
	float Beita_02=3*LADRC_param_One.w0*LADRC_param_One.w0;
	float Beita_03=LADRC_param_One.w0*LADRC_param_One.w0*LADRC_param_One.w0;


	float e= LADRC_param_One.z1-FeedBack;
	LADRC_param_One.z1+= (LADRC_param_One.z2 - Beita_01*e)*LADRC_param_One.h;
	LADRC_param_One.z2+= (LADRC_param_One.z3 - Beita_02*e + LADRC_param_One.b0*LADRC_param_One.u)*LADRC_param_One.h;
	LADRC_param_One.z3+=-Beita_03*e*LADRC_param_One.h;
}

float LADRC_Control(LADRC_param LADRC_param_One , float Expect_Value , float Measue)//控制量求解
{

	//LADRC_TD(LADRC_param_One , Expect_Value);
	LADRC_ESO(LADRC_param_One , Measue);
	//float Kp=LADRC_param_One.wc*LADRC_param_One.wc;
	//float Kd=2*LADRC_param_One.wc;
	float e1=Expect_Value-LADRC_param_One.z1;
	float e2=LADRC_param_One.z2;
	float u0=LADRC_param_One.Kp*e1+LADRC_param_One.Kd*e2;
	LADRC_param_One.u=(u0-LADRC_param_One.z3)/LADRC_param_One.b0;

	return LADRC_param_One.u;
}
