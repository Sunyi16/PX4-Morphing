#include<string.h>
#include<px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include<drivers/drv_pwm_output.h>
#include<lib/cdev/CDev.hpp>
#include<lib/systemlib/err.h>
#include<drivers/pwm_out/PWMOut.hpp>
//#include <px4_platform_common/getopt.h>
//#include <px4_platform_common/posix.h>
using namespace std;

extern "C" __EXPORT int steering_engine_main(int argc, char *argv[]);

class Steering_engine : public ModuleBase<Steering_engine>,  public ModuleParams
{
public:
	Steering_engine();
	~Steering_engine() override;
	static int task_spawn(int argc, char*argv[]);
	void run()override;
	static Steering_engine *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
};
PWMOut qw;
int Steering_engine::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

Steering_engine::Steering_engine():
	ModuleParams(nullptr)
{
}
Steering_engine::~Steering_engine()
{
	PX4_WARN("hased");
}
int Steering_engine::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("steering_engine",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}
Steering_engine *Steering_engine::instantiate(int argc, char *argv[])
{
	Steering_engine *instance=new Steering_engine();
	if(instance==nullptr)
	{
		PX4_ERR("alloc failed");
	}
	return instance;
}

void Steering_engine::run()
{
/*	for(int i=0;i<10;i++)
	{
		PX4_WARN("%d",i);
		sleep(1);
	}

	int	fd=open(PX4FMU_DEVICE_PATH,O_RDWR);
	if(fd<0){
		errx(1,"open fail");
	}
	int i=6;
	int ret=ioctl(fd,PWM_SERVO_SET(i),1800);
	if(ret!=OK){
		PX4_ERR("PWM_SERVO_SET(%d)",i);
	}

	int asd=qw.updateOutputs(false,outputs[0],1,0);
	if(asd){
		PX4_INFO("runing");
	}
*/
up_pwm_servo_set(9,1800);
PX4_INFO("running");


}

int Steering_engine::print_usage(const char *reason )
{
	PX4_WARN("first start/stop");
	if (reason) {
		PX4_WARN("%s\n", reason);
	}
	return 0;
}

int steering_engine_main(int argc, char*argv[])
{
	return Steering_engine::main(argc,argv);
}
