/**
 * @file smc_eso.cpp
 *
 * Implementation of generic SMC_eso controller.
 *
 * @author sunyi
 */

#include"smc_eso.h"
#include <uORB/topics/parameter_update.h>

/**
 * 用于设计趋近率的函数，此处采用指数趋近率，使用符号函数
*/

int smc::sign(float s)
{
	if (s<0)
	{
		return -1;
	}
	else if (s>0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

float smc::sat(float s)
{
	float compare_val = 10;
	if (s<-compare_val)
	{
		return -1;
	}
	else if (s>compare_val)
	{
		return 1;
	}
	else
	{
		return s/compare_val;
	}
}

float smc::Constrain_Float(float amt, float low, float high)
{
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

smc::smc()
{
	smc::c=1;		//滑膜面系数*
	smc::c2=1;
	smc::Ixx=0.05;		//无人机三轴惯量
	smc::Iyy=0.05;
	smc::Izz=0.08;
	smc::x=1;		//指数趋近率系数，x为符号项
	smc::k=1;
	smc::b0=35;		//扰动补偿增益
	smc::k2=1;		//误差微分增益
	smc::r1=4;		//超螺旋系数
	smc::r2=5;

	ADRC_Init(&smc_eso_pitch,&smc_eso_roll,&smc_eso_yaw);//ESO初始化

	smc::last_err(0) = 0;		//中间状态初始化
	smc::last_err(1) = 0;
	smc::last_err(2) = 0;
	smc::last_rate_sp(0) = 0;
	smc::last_rate_sp(1) = 0;
	smc::last_rate_sp(2) = 0;
	smc::last_att_sp(0) = 0;
	smc::last_att_sp(1) = 0;
	smc::last_att_sp(2) = 0;
	smc::last_datt_sp(0) = 0;
	smc::last_datt_sp(1) = 0;
	smc::last_datt_sp(2) = 0;
	smc::integral(0) = 0;
	smc::integral(1) = 0;
	smc::integral(2) = 0;
}

smc::~smc()
{
}

matrix::Vector3f smc::SMC_Control(Vector3f att_sp , Vector3f att , Vector3f rate ,float dt)
{
	Vector3f err = att - att_sp;
	//Vector3f derr = (err-smc::last_err)/dt;
	Vector3f s = smc::c*err + smc::c2*(err -last_err)/dt;		//为三个姿态设计滑模面
	//Vector3f s2 = err + (err -last_err)/dt;		//为三个姿态设计滑模面
	Vector3f datt_sp = (att_sp - smc::last_att_sp)/dt;
	Vector3f ddatt_sp = (datt_sp - smc::last_datt_sp)/dt;


	/***********耦合项***********/
	float f_roll = ddatt_sp(0)-rate(1)*rate(2)*(smc::Iyy-smc::Izz)/smc::Ixx;
	float f_pitch = ddatt_sp(1)-rate(0)*rate(2)*(smc::Izz-smc::Ixx)/smc::Iyy;
	float f_yaw = ddatt_sp(2)-rate(1)*rate(0)*(smc::Ixx-smc::Iyy)/smc::Izz;

	smc::ESO_Z3(att);//ESO观测干扰

	Vector3f torque;

	/************BSTSMC-ESO************/
/* 	torque(0)= smc::Ixx*(smc::eso_z3(0)/smc::b0-ddatt_sp(0)-smc::k2*derr(0)-smc::k*s(0)-err(0)-smc::r1*sqrt(abs(s(0)))*sat(s(0))-smc::r2*smc::integral(0));
	torque(1)= smc::Iyy*(smc::eso_z3(1)/smc::b0-ddatt_sp(1)-smc::k2*derr(1)-smc::k*s(1)-err(1)-smc::r1*sqrt(abs(s(1)))*sat(s(1))-smc::r2*smc::integral(1));
	torque(2)= smc::Izz*(smc::eso_z3(2)/smc::b0-ddatt_sp(2)-smc::k2*derr(2)-smc::k*s(2)-err(2)-smc::r1*sqrt(abs(s(2)))*sat(s(2))-smc::r2*smc::integral(2)); */

	/*********BC-ESO**********/
/* 	torque(0)= smc::Ixx*(smc::eso_z3(0)/smc::b0-ddatt_sp(0)-smc::k2*derr(0)-smc::k*s(0)-err(0));
	torque(1)= smc::Iyy*(smc::eso_z3(1)/smc::b0-ddatt_sp(1)-smc::k2*derr(1)-smc::k*s(1)-err(1));
	torque(2)= smc::Izz*(smc::eso_z3(2)/smc::b0-ddatt_sp(2)-smc::k2*derr(2)-smc::k*s(2)-err(2));  */

	/***********BSMC-ESO***********/
	/* torque(0)= smc::Ixx*(smc::eso_z3(0)/smc::b0-ddatt_sp(0)+datt_sp(0)-rate(0)-smc::k*s(0)-err(0)+smc::x*sat(s(0)));
	torque(1)= smc::Iyy*(smc::eso_z3(1)/smc::b0-ddatt_sp(1)+datt_sp(1)-rate(1)-smc::k*s(1)-err(1)+smc::x*sat(s(1)));
	torque(2)= smc::Izz*(smc::eso_z3(2)/smc::b0-ddatt_sp(2)+datt_sp(2)-rate(2)-smc::k*s(2)-err(2)+smc::x*sat(s(2))); */

	torque(0)= smc::Ixx*(smc::eso_z3(0)/smc::b0+f_roll-ddatt_sp(0)+datt_sp(0)-rate(0)-smc::k*s(0)-err(0)-smc::r1*(float)std::sqrt(abs(s(0)))*sat(s(0))-smc::r2*smc::integral(0));
	torque(1)= smc::Iyy*(smc::eso_z3(1)/smc::b0+f_pitch-ddatt_sp(1)+datt_sp(1)-rate(1)-smc::k*s(1)-err(1)-smc::r1*(float)std::sqrt(abs(s(1)))*sat(s(1))-smc::r2*smc::integral(1));
	torque(2)= smc::Izz*(smc::eso_z3(2)/smc::b0+f_yaw-ddatt_sp(2)+datt_sp(2)-rate(2)-smc::k*s(2)-err(2)-smc::r1*(float)std::sqrt(abs(s(2)))*sat(s(2))-smc::r2*smc::integral(2));

/* 	torque(0)= smc::Ixx*(smc::eso_z3(0)/smc::b0-ddatt_sp(0)+datt_sp(0)-rate(0)-smc::k*s(0)-err(0)+smc::x*sat(s(0)));
	torque(1)= smc::Iyy*(smc::eso_z3(1)/smc::b0-ddatt_sp(1)+datt_sp(1)-rate(1)-smc::k*s(1)-err(1)+smc::x*sat(s(1)));
	torque(2)= smc::Izz*(smc::eso_z3(2)/smc::b0-ddatt_sp(2)+datt_sp(2)-rate(2)-smc::k*s(2)-err(2)+smc::x*sat(s(2)));
 */

	//smc::ESO_Z3(att);//ESO观测干扰

	/*************补偿干扰*************/
/* 	torque(0)=torque(0)-smc::eso_z3(0)/smc::b0;
	torque(1)=torque(1)-smc::eso_z3(1)/smc::b0;
	torque(2)=torque(2)-smc::eso_z3(2)/smc::b0; */


	/****************控制器输出限幅****************/
	torque(0)=smc::Constrain_Float(torque(0),-450,450);
	torque(1)=smc::Constrain_Float(torque(1),-450,450);
	torque(2)=smc::Constrain_Float(torque(2),-450,450);

	smc::last_torque=torque;//控制器输出传递到ESO输入

	smc::last_att_sp=att_sp;
	smc::last_datt_sp=datt_sp;
	smc::last_err=err;
	smc::integral(0)+=sat(s(0))*dt;
	smc::integral(1)+=sat(s(1))*dt;
	smc::integral(2)+=sat(s(2))*dt;

	return torque;


}

//获取ESO干扰观测值
void  smc::ESO_Z3(Vector3f att)
{
	smc_eso_roll.y=att(0);
	smc_eso_pitch.y=att(1);
	smc_eso_yaw.y=att(2);

	smc_eso_roll.u=smc::last_torque(0);
	smc_eso_pitch.u =smc::last_torque(1);
	smc_eso_yaw.u  =smc::last_torque(2);

	ESO_ADRC(&smc_eso_roll);
	ESO_ADRC(&smc_eso_pitch);
	ESO_ADRC(&smc_eso_yaw);

	smc::eso_z3(0)=smc_eso_roll.z3;
	smc::eso_z3(1)=smc_eso_pitch.z3;
	smc::eso_z3(2)=smc_eso_yaw.z3;



}
