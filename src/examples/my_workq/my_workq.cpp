#include<uORB/uORB.h>
#include<msg/tmp/headers/actuator_controls.h>
#include<px4_platform_common/module.h>
#include<px4_platform_common/module_params.h>
#include<px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include<msg/tmp/headers/manual_control_setpoint.h>

using namespace std;
extern "C" __EXPORT int my_workq_main(int argc, char *argv[]);

class my_workq : public ModuleBase<my_workq>,public ModuleParams,public px4::ScheduledWorkItem
{
private:
	void Run() override;

public:
	my_workq();
	~my_workq() override;
	static int task_spawn(int argc, char* argv[]);
	static int custom_command(int argc,char* argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

	bool	_actuators_2_circuit_breaker_enabled;
	orb_advert_t _actuators_2_pub;
	struct actuator_controls_s	_actuators2;
	int ma_co_set_sub_fd = orb_subscribe(ORB_ID(manual_control_setpoint));
	struct manual_control_setpoint_s mcs;
	orb_id_t _actuators_id2 =  ORB_ID(actuator_controls_2);

};

my_workq::my_workq():
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}
my_workq::~my_workq()
{
	PX4_WARN("hasd");
}
bool my_workq::init()
{
	ScheduleOnInterval(1000);
	return 1;
}
void my_workq::Run()
{
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

}
}
int my_workq::task_spawn(int argc, char*argv[])
{
	my_workq *instance=new my_workq();

	if(instance)
	{
		_object.store(instance);
		_task_id=task_id_is_work_queue;

		if(instance->init())
		{
			return PX4_OK;
		}
	}
	else
	{
		PX4_ERR("alloc failed");
	}
	delete instance;
	_object.store(nullptr);
	_task_id=1;

	return PX4_ERROR;
}
int my_workq::print_status()
{
	return 0;
}
int my_workq::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int my_workq::print_usage(const char *reason)
{
	if(reason)
	{
		PX4_WARN("%s",reason);
	}
	return 0;
}
int my_workq_main(int argc, char *argv[])
{
	return my_workq::main(argc , argv);
}
