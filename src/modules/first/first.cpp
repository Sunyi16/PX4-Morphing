#include<uORB/uORB.h>
#include"first.h"
#include<drivers/drv_hrt.h>
#include<msg/tmp/headers/manual_control_setpoint.h>
#include"uart.h"


using namespace matrix;
using namespace std;
extern "C" __EXPORT int first_main(int argc, char *argv[]);

int First::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

First::	First():
	ModuleParams(nullptr)
{
	/* _actuators_2_pub = nullptr;
	_actuators_id2 = nullptr;
	_actuators_2_circuit_breaker_enabled = false;
	memset(&_actuators2, 0 ,sizeof(_actuators2));//结构体清零 */

}
First::~First()
{
	PX4_WARN("hased");
}
int First::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("first",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2000,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return 0;
	}

	return 0;
}
First *First::instantiate(int argc, char *argv[])
{
	First *instance=new First();
	if(instance==nullptr)
	{
		PX4_ERR("alloc failed");
	}
	return instance;
}

void First::run()
{
/* 	int ma_co_set_sub_fd = orb_subscribe(ORB_ID(manual_control_setpoint));
	struct manual_control_setpoint_s mcs;
	_actuators_id2 =  ORB_ID(actuator_controls_2);
	while (!PX4_OK)
	{
	orb_copy(ORB_ID(manual_control_setpoint) , ma_co_set_sub_fd , &mcs);
	if((double)mcs.aux2>0.0)
	{
	if( (double)mcs.aux1>0.0)
	{
	_actuators2.control[4] = -1.0f;//-30
	_actuators2.control[5] = -1.0f;
	_actuators2.control[6] = -1.0f;
	_actuators2.control[7] = -1.0f;
	}
	else
	{
	_actuators2.control[4] = -0.5f;//1500，对应舵机0度
	_actuators2.control[5] = -0.5f;
	_actuators2.control[6] = -0.5f;
	_actuators2.control[7] = -0.5f;
	}}
	else
	{
	_actuators2.control[4] = 0.2f;//60
	_actuators2.control[5] = 0.2f;
	_actuators2.control[6] = -1.0f;//-30
	_actuators2.control[7] = -1.0f;
	}
	//_actuators2.control[7] = 1.0f;//对应2000us最高值
	_actuators2.timestamp = hrt_absolute_time();
	//_actuators2.timestamp_sample = _ctrl_state.timestamp;


	if(!_actuators_2_circuit_breaker_enabled){
		if(_actuators_2_pub!=nullptr){
			orb_publish(_actuators_id2,_actuators_2_pub,&_actuators2);
			//PX4_INFO("runing");
		}else if(_actuators_id2){
			_actuators_2_pub=orb_advertise(_actuators_id2, &_actuators2);
		}
	}
	} */
	int succe;

	int uart_read = uart_init((char*)"/dev/ttyS6");
        if(false == uart_read)succe = -1;
        if(false == set_uart_baudrate(uart_read,115200)){
	printf("%f",(double)succe);
     //   printf("[YCM]set_uart_baudrate is failed\n");

        }
	while(1){
	_vehicle_attitude_sub.update(&_v_att);

        float pitch = Eulerf(Quatf(_v_att.q)).theta();


        printf("%f\n",(double)pitch);

	int mid1=1500+(int)(pitch*1000);
	char con[5]={'\0'};
	con[0]=(char)('0'+mid1/1000);
	con[1]=(char)('0'+mid1%1000/100);
	con[2]=(char)('0'+mid1%100/10);
	con[3]=(char)('0'+mid1%10);


        char con1_write[16] ="#001P2000T0000!";
	con1_write[5]=con[0];
	con1_write[6]=con[1];
	con1_write[7]=con[2];
	con1_write[8]=con[3];

	write(uart_read,&con1_write,15);
	//px4_sleep(0.5);

	}

	}



int First::print_usage(const char *reason )
{
	PX4_WARN("first start/stop");
	if (reason) {
		PX4_WARN("%s\n", reason);
	}
	return 0;
}

int first_main(int argc, char*argv[])
{

	return First::main(argc,argv);
}
