/**
 * @file smc.cpp
 *
 * Implementation of generic SMC controller.
 *
 * @author sunyi
 */

#include"smc.h"

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

smc::smc()
{
	smc::c=1;
	smc::Ixx=1;
	smc::Iyy=1;
	smc::Izz=1;
	smc::x=1;
	smc::k=1;

	smc::last_err(0) = 0;
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
}

smc::~smc()
{
}

matrix::Vector3f smc::SMC_Control(Vector3f att_sp , Vector3f att , Vector3f rate ,float dt)
{
	Vector3f err = att_sp - att;
	Vector3f derr = (err-smc::last_err)/dt;
	Vector3f s = smc.c err + (err - last_err)/dt;		//为三个姿态设计滑模面
	Vector3f datt_sp = (att_sp - smc::last_att_sp)/dt;
	Vector3f ddatt_sp = (datt_sp - smc::last_datt_sp)/dt;

	float f_roll = ddatt_sp(0)-rate(1)*rate(2)*(smc::Iyy-smc::Izz)/smc::Ixx;
	float f_pitch = ddatt_sp(1)-rate(0)*rate(2)*(smc::Izz-smc::Ixx)/smc::Iyy;
	float f_yaw = ddatt_sp(2)-rate(1)*rate(0)*(smc::Ixx-smc::Iyy)/smc::Izz;

	Vector3f torque;
	torque(0)= smc::Ixx*(ddatt_sp(0)+datt_sp(0)-rate(0)-f_roll-smc::x*sgn(s(0))-smc::k*s(0));
	torque(1)= smc::Iyy*(ddatt_sp(1)+datt_sp(1)-rate(1)-f_pitch-smc::x*sgn(s(1))-smc::k*s(1));
	torque(2)= smc::Izz*(ddatt_sp(2)+datt_sp(2)-rate(2)-f_pitch-smc::x*sgn(s(2))-smc::k*s(2);

	return torque;


}
