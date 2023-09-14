/**
 * @file smc_eso.h
 *
 * Definition of generic SMC_ESO controller.
 *
 * @author Sunyi
 */

#ifndef SMC_ESO_H_
#define SMC_ESO_H_

#include <stdint.h>
#include <matrix/matrix/math.hpp>
#include <lib/adrc/adrc.h>
using namespace matrix;

__BEGIN_DECLS
class smc
{
private:
	float c;	//滑膜面系数
	float c2;
	float Ixx;	//惯量
	float Iyy;
	float Izz;
	float x;	//趋近率系数
	float k;
	float b0;
	float k2;
	float r1;
	float r2;

	Fhan_Data smc_eso_pitch;
	Fhan_Data smc_eso_roll;
	Fhan_Data smc_eso_yaw;

	Vector3f last_err;
	Vector3f last_rate_sp;
	Vector3f last_att_sp;
	Vector3f last_datt_sp;
	Vector3f last_torque;
	Vector3f eso_z3;
	Vector3f integral;
public:
	smc();
	~smc();
	int sign(float s);		//符号函数
	float sat(float s);		//饱和函数
	float Constrain_Float(float amt, float low, float high);
	void ESO_Z3(Vector3f att);

	matrix::Vector3f SMC_Control(Vector3f att_sp , Vector3f att , Vector3f rate ,float dt);

};


__END_DECLS
#endif /* PID_H_ */
