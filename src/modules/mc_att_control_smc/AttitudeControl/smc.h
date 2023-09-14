/**
 * @file smc.h
 *
 * Definition of generic SMC controller.
 *
 * @author Sunyi
 */

#ifndef SMC_H_
#define SMC_H_

#include <stdint.h>
#include <matrix/matrix/math.hpp>
#include <lib/adrc/adrc.h>
using namespace matrix;

__BEGIN_DECLS
class smc
{
private:
	float c;	//滑膜面系数
	float Ixx;	//惯量
	float Iyy;
	float Izz;
	float x;	//趋近率系数
	float k;

	Vector3f last_err;
	Vector3f last_rate_sp;
	Vector3f last_att_sp;
	Vector3f last_datt_sp;
public:
	smc();
	~smc();
	int sign(float s);		//符号函数
	float sat(float s);		//饱和函数
	float Constrain_Float(float amt, float low, float high);

	matrix::Vector3f SMC_Control(Vector3f att_sp , Vector3f att , Vector3f rate ,float dt);

};

__END_DECLS
#endif /* PID_H_ */
