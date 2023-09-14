#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include<px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include<string.h>

__EXPORT int rw_uart_main(int argc, char *argv[]);

static int uart_init(char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);

int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            PX4_WARN("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config);
    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;
    /* no parity, one stop bit */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        PX4_WARN("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        PX4_WARN("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        PX4_WARN("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int uart_init(char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

/*     if (serial_fd < 0) {
        PX4_ERR(1, "failed to open port: %s", uart_name);
        return false;
    } */
    return serial_fd;
}

int rw_uart_main(int argc, char *argv[])
{
    char data = '0';
    char con0_write[15] ="#000P2000T1000!";
    char con1_write[15] ="#001P2000T0000!";
    char con2_write[15] ="#002P2000T0000!";
    char con3_write[15] ="#003P2000T0000!";
    char con4_write[15] ="#004P2000T0000!";
    char con5_write[15] ="#005P2000T0000!";

    char con0_write_ID[9] ="#000PRAD!";



   char buffer[30] = "0";
    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
    int uart_read = uart_init("/dev/ttyS6");
    if(false == uart_read)return -1;
    if(false == set_uart_baudrate(uart_read,115200)){
        printf("[YCM]set_uart_baudrate is failed\n");
        return -1;
    }
    printf("[YCM]uart init is successful\n");

 //   while(true){
//	float retu = write(uart_read,con_write,14);
 //       read(uart_read,&data,1);
//        if(data == 'R'){
/*              for(int i = 0;i <4;++i){
		write(uart_read,&con_write,15);
                read(uart_read,&data,1);
                buffer[i] = data;
                data = '0'; */
//		printf("%s\n",buffer);
 //           }
    write(uart_read,&con0_write,15);

    px4_sleep(2);

    write(uart_read,&con1_write,15);
    px4_sleep(2);

    write(uart_read,&con2_write,15);
    px4_sleep(2);

    write(uart_read,&con3_write,15);
    px4_sleep(2);

    write(uart_read,&con4_write,15);
    px4_sleep(2);

    write(uart_read,&con5_write,15);

    write(uart_read,&con0_write_ID,9);

  //  read(uart_read,&data,1);
  //      if(data == 'R'){
              for(int i = 0;i <9;++i){
                read(uart_read,&data,1);
                buffer[i] = data;
                data = '0'; }
//		printf("%s\n",buffer);

            printf("%s\n",buffer);
 //       }
//	else
//	printf("It is %f",(double)retu);
 //   }

    return 0;
}
