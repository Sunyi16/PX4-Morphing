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
	int succe;

	int uart_read = uart_init((char*)"/dev/ttyS6");
        if(false == uart_read)succe = -1;
        if(false == set_uart_baudrate(uart_read,115200)){
	printf("12%f",(double)succe);
     //   printf("[YCM]set_uart_baudrate is failed\n");

        }

	while(1)
	{

	int ser_sub_fd = orb_subscribe(ORB_ID(ser_ver));
//	bool update;
//	orb_check(ser_sub_fd, &update);//检查更新
	orb_copy(ORB_ID(ser_ver),ser_sub_fd,&ser_ver)
	/*定义六个舵机的控制角度与读取角度信息命令*/

        char con0_write[16] =ser_ver.ser0;//1号舵机，P和T之间为驱动PWM值
        char con0_write1[10] ="#000PRAD!";//读取对应舵机的角度

	char con1_write[16] =ser_ver.ser1;
        char con1_write1[10] ="#001PRAD!";

	char con2_write[16] =ser_ver.ser2;
        char con2_write1[10] ="#002PRAD!";

	char con3_write[16] =ser_ver.ser3;
        char con3_write1[10] ="#003PRAD!";

	char con4_write[16] =ser_ver.ser4;
        char con4_write1[10] ="#004PRAD!";

	char con5_write[16] =ser_ver.ser5;
        char con5_write1[10] ="#005PRAD!";


	orb_advert_t ser_pub = orb_advertise(ORB_ID(deg_ser),&deg);

	deg.deg0 = write_read_ser(uart_read , con0_write , con0_write1);
	deg.deg1 = write_read_ser(uart_read , con1_write , con1_write1);
	deg.deg2 = write_read_ser(uart_read , con2_write , con2_write1);
	deg.deg3 = write_read_ser(uart_read , con3_write , con3_write1);
	deg.deg4 = write_read_ser(uart_read , con4_write , con4_write1);
	deg.deg5 = write_read_ser(uart_read , con5_write , con5_write1);

	orb_publish(ORB_ID(deg_ser), ser_pub , &deg);

	}

	}


int First::write_read_ser(int uart_read ,char *con0_write , char *con0_write1)
{

	char data = '0';
	char buffer[30] = "0";
	write(uart_read,&con0_write,15);
	usleep(5000);
	write(uart_read,&con0_write1,9);
	/*读程序*/

	usleep(5000);
        for(int i = 0;i <10;i++){
		read(uart_read,&data,1);
                buffer[i] = data;
                data = '0';
		usleep(5000);}
	int ser0 = buffer[5]*1000+buffer[6]*100+buffer[7]*10+buffer[8];
	int ser_deg0 = (int)((ser0-1500)*0.135);
	return ser_deg0;
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
