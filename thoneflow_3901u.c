#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/optical_flow.h>
//#include <uORB/topics/vehicle_attitude.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_shutdown.h>
#include <px4_tasks.h>
#include <px4_time.h>



/**
 * @file thoneflow.c
 * @author Tonser <sundxfansky@sjtu.edu.cn>, Stone White <stone@thone.io>
 * v1.0 2019.5.20
 *
 * ThoneFlow-3901U光流驱动 通过串口传递数据 发布ORB_ID(optical_flow)
 *
 * This driver publish optical flow data.
 */

//#define  angle_to_rad  0.0174f //角度转弧度
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;


__EXPORT int thoneflow_main(int argc, char *argv[]);
int thoneflow_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud); //static
static void usage(const char *reason);            //static
//extern orb_advert_t mavlink_log_pub_thoneflow;
orb_advert_t mavlink_log_pub_thoneflow = NULL;

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
            PX4_INFO("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;


    tcgetattr(fd, &uart_config);

    uart_config.c_oflag &= ~ONLCR;

    uart_config.c_cflag &= ~(CSTOPB | PARENB);

    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        PX4_INFO("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        PX4_INFO("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        PX4_INFO("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int uart_init(const char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}

static void usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "WARN: lose para,use {start|stop|status} [param]\n\n");
    exit(1);
}

int thoneflow_main(int argc, char *argv[])
{

mavlink_log_info(&mavlink_log_pub_thoneflow,"[inav] thoneflow_main on init");
    if (argc < 2) 
    {
        usage("[YCM]missing command");
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            PX4_INFO("[YCM]already running\n");
            exit(0);
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("thoneflow",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         2500,
                         thoneflow_thread_main,
                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        thread_running = false;
        return 0;
    }
    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("[YCM]running");

        } else {
            warnx("[YCM]stopped");
        }

       return 0;
    }

    usage("unrecognized command");
    return 1;
}

int thoneflow_thread_main(int argc, char *argv[])
{
    mavlink_log_info(&mavlink_log_pub_thoneflow,"thoneflow run ");
    char data = '0';
    char buffer[7] = "";
    char sum = '0';
    float thoneflow_x = -1;
    float thoneflow_y = -1;
    int uart_read = uart_init("/dev/ttyS3"); // ttyS3 for fmuv5 ttyS6 for fmuv2

    if(false == uart_read)
    {
         mavlink_log_critical(&mavlink_log_pub_thoneflow,"[YCM]uart init is failed\n");
         return -1;
    }
    if(false == set_uart_baudrate(uart_read,19200)){
        return -1;
    }
    mavlink_log_info(&mavlink_log_pub_thoneflow,"[YCM]uart init is successful\n");
    thread_running = true;
    // 定义话题结构
    struct optical_flow_s flow_data;
    memset(&flow_data, 0 , sizeof(flow_data));
    //公告消息
    orb_advert_t flow_data_handle = orb_advertise(ORB_ID(optical_flow), &flow_data);//公告这个主题
    // int counter = 0;
    uint64_t _previous_collect_timestamp = hrt_absolute_time();
    uint64_t _flow_dt_sum_usec = 0;
    float scale = 3.52f;// need to be adjust
    while(thread_running)
   {
        //解码 串口信息
    	read(uart_read,&data,1);
        if(data == 0xFE)
        {
            data = '0';
            read(uart_read,&data,1);
            if(data == 0x04)
            {
                for(int k = 0;k < 7;++k)
                {
                data = '0';
                read(uart_read,&data,1);
                buffer[k] = data;
                }
                sum = buffer[0]+buffer[1]+buffer[2]+buffer[3];
                if (buffer[4]==sum&&buffer[6]==0xAA){
                    uint64_t timestamp = hrt_absolute_time();
                    uint64_t dt_flow = timestamp - _previous_collect_timestamp;
                    _previous_collect_timestamp = timestamp;
                    _flow_dt_sum_usec += dt_flow;
                    thoneflow_x = (float)((int16_t)(buffer[1]<<8|buffer[0])/1000.0f);// 单位是m
                    thoneflow_y = (float)((int16_t)(buffer[3]<<8|buffer[2])/1000.0f);// 单位是m
                    flow_data.pixel_flow_x_integral = thoneflow_x*scale;
                    flow_data.pixel_flow_y_integral = thoneflow_y*scale;
                    flow_data.gyro_x_rate_integral = 0.0f;
                    flow_data.gyro_y_rate_integral = 0.0f;
                    flow_data.gyro_z_rate_integral = 0.0f;
                    flow_data.min_ground_distance = 0.08;//Minimum distance to 8cm
                    flow_data.max_ground_distance = 8;//Depends on Rangefinder
                    flow_data.max_flow_rate = 2.5;
                    flow_data.sensor_id = 0;
                    flow_data.timestamp = timestamp;
                    flow_data.frame_count_since_last_readout = 1; //4;
                    flow_data.integration_timespan = _flow_dt_sum_usec;
                    _flow_dt_sum_usec = 0;
                    if ((int16_t)buffer[5]<64){buffer[5]=64;}
                    if ((int16_t)buffer[5]>78){buffer[5]=78;}
                    flow_data.quality = (buffer[5]-64)*255/14;
                    orb_publish(ORB_ID(optical_flow),flow_data_handle,  &flow_data);
                }
            }
        }
   }
    thread_running = false;
    close(uart_read);
    fflush(stdout);
    return 0;

}
