/**
 * @file ladrc.h
 *
 * Definition of generic LADRC controller.
 *
 * @author Sunyi
 */

#ifndef LADRC_H_
#define LADRC_H_

#include <stdint.h>

__BEGIN_DECLS
typedef struct LADRC
{
	float v1 , v2;		//最速输出值
	float r;	//速度因子
	float h;	//步长
	float z1 , z2 , z3;		//观测器输出
	float w0 , wc , b0 , u;
	float Kp,Kd;
}LADRC_param;


void LADRC_init(LADRC_param *LADRC_param_Pitch , LADRC_param *LADRC_param_Roll , LADRC_param *LADRC_param_Yaw);//参数初始化
void LADRC_TD(LADRC_param LADRC_param_One , float Expect);//跟踪微分部分
void LADRC_ESO(LADRC_param LADRC_param_One , float FeedBack);//状态哦观测器
float LADRC_Control(LADRC_param LADRC_param_One , float Expect_Value , float Measue);//控制量求解




__END_DECLS
#endif /* PID_H_ */
