// Title: Wj6 Hinson Modbus RTU Driver with Odometry 
// Author: Dr. Chen Chun-Lin for Danbach Modbus RTU and added odometry with speed read filter
// Modified by: Ankur Gupta for Wj6 Hinson Modbus RTU and added odometry
// Modified date: 2020/11/03, 2020/11/18, 2020/11/24-26, 2020/12/09

// Subscribed topic : agv_cmd_vel
// Published topic : odom,
// Published TF: odom -> base_link

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sstream>

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <cmath>     /* D-DWM-PG2.5 angle calculation */

#include <sys/types.h>      /* UDP transmission */
#include <sys/socket.h>     /* UDP transmission */
#include <netinet/in.h>     /* UDP transmission */


#include <wj6_motor_driver/MotorsSpeed.h>

#define CRC16 0x8005

#define REPLY_SIZE 1
#define TIMEOUT 1000

#define PI      3.14159  //Ratio of the circumference of a circle to its diameter
#define RTD     180.0/PI //Radian to Degree
#define DTR     PI/180.0 //Degree to Radian
#define btoa(x) ((x)?"true":"false")
//ROS Parameter Default
#define MAIN_FREQ          400.0          //Controller main frequency --400.0(crc check fail)
#define TF_REV_DELAY_T     3.0            //TF of MAP_ODOM frame receive start delay units * ts
#define MODBUS_CMD_SEND_DELAY_T 0.05      //unit:s, 0: not appcation, >0: appcation
#define MODBUS_CMD_RESEND_MAX 0         //0: not appcation, >0: appcation
#define MODBUS_CMD_RESEND_TIME 0.5      //unit:s
#define MODBUS_RECVEIE_TIMEOUT 1.0      //unit:s
#define FRAME_BUFF_MAX 64

#define SERVER_PORT 8888    // UDP port
#define BUFF_LEN 1024       // UDP buffer


//------------------------------------------
unsigned char data[6];

//------------------------------------------
//ROS Parameter
double main_freq_=MAIN_FREQ ;
double modbus_cmd_send_delay_t_=MODBUS_CMD_SEND_DELAY_T;
std::string modbus_port_;
std::string modbus_baud_;
double modbus_cmd_resend_max_=MODBUS_CMD_RESEND_MAX;
double modbus_cmd_resend_time_=MODBUS_CMD_RESEND_TIME;
double modbus_recveie_timeout_=MODBUS_RECVEIE_TIMEOUT;

//---------------------------------------------------
//for UART recveier time-out setting
fd_set rset;
int ret = 0;
struct timeval tv;

//---------------------------------------------------
int rev_frame_size=0;
int frame_sent=0;
int frame_recved=0;
int frame_checked=0;
unsigned char frame_bytes[FRAME_BUFF_MAX];
unsigned int frame_idx=0;
unsigned short frame_crc16=0;
int frame_resend_cnt=0;
int trans_step=0;
int trans_next=0;
static int get_new_cmd=0;
static short trans_vLa=0;
static short trans_vRa=0;
//------------------------------------------

static char fault_clear = 1;

static char temp_count = 0;


int count = 0;
int fd; /* File descriptor for the port */
unsigned buffer[1000];
unsigned int buf_cnt=0;
bool frame_get_flag=0;
unsigned int frame_rev_freq_cnt=0;
unsigned char read_bytes[800];
unsigned int read_idx=0;
bool syn_flag=0;
int syn_num=0;

ros::Time current_ctime, last_ctime;
double cdt=0;
bool get_ctime=0;

//Odometry
ros::Time time1, time2, odom_current_time, odom_last_time;
ros::Time current_time;
double x = 0.0;
double y = 0.0;
double th = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
bool get_time1=0;
bool get_time2=0;
bool get_dt=0;

//------------------------------------------------

//D-DWM-PG2.5 
int tag_id = 0;
short int x_tag0 = 0;
short int y_tag0 = 0;
short int x_tag1 = 0;
short int y_tag1 = 0;
short int x_tag2 = 0;
short int y_tag2 = 0;
static int pos_check = 0; //only equal to 0x010F = 271 when uwb senses the correct data

//UDP transmission
int udp_init = -1;
int server_fd, ret_;
struct sockaddr_in ser_addr;

int count_;
struct sockaddr_in clent_addr;  //clent_addr用于记录发送方的地址信息

unsigned short crc_fn(unsigned char *dpacket, unsigned int len) // CRC Function(Error calcualtion)
{
    unsigned short crc = 0xffff,poly = 0xa001; //Modbus CRC-16
    unsigned short i=0;

    for(i=0;i<len;i++)
    {
        crc^= dpacket[i];
        for(int j=0;j<8;j++)
        {
            if(crc & 0x01)
            {
                crc >>= 1;
                crc ^= poly;
            }
            else
                crc >>= 1;
        }
    }
    return (crc);
}

void write_modbus(unsigned char *data, unsigned int len)
{
    unsigned short crc16;
    unsigned char buff[40];
    unsigned char crc16_data[2];
    unsigned short i=0;
    unsigned short j=0;

    for (i=0;i<len;i++)
        buff[i]=data[i];
    crc16=crc_fn(&buff[0], len); //modbus CRC16
    crc16_data[0]=(unsigned char)(crc16 & 0xff);
    crc16_data[1]=(unsigned char)((crc16 >> 8) & 0xff);
    buff[len+1-1]=crc16_data[0];
    buff[len+2-1]=crc16_data[1];
    //printf("crc16=%d\n", crc16);
    //printf("crc16_data[0]=0x%02X, crc16_data[1]=0x%02X\n", crc16_data[0], crc16_data[1]);
    //printf("Mobus Data Sent:0x"); 
    //for (j=0;j<len;j++)
    //    printf(" %02X", buff[j]);
    //printf(" %02X %02X\n", crc16_data[0], crc16_data[1]);
    write(fd, &buff[0], len+2); //with CRC16
}

void uwb_pos_cmd()
{
    printf("fault_clear = %d\n",fault_clear);
    unsigned short addr = 0x01; //ADR
    unsigned char buff[40];
 /**
    //Modbus-RTU Header:
    buff[0] = addr; //255 communication ID (Slave ID Address)
    buff[1] = 0x10; //function code 0x10 write to register(s)
    buff[2] = 0x00; //address for starting register 
    buff[3] = 0x28;
    buff[4] = 0x00; //amount of registers used
    buff[5] = 0x01; 
    buff[6] = 0x02; //length of data bytes (amount of registers used *2)
    buff[7] = 0x00; //data to be written into register 1
    buff[8] = 0x01;
 //    buff[9] = 0x00; //data to be written into register 2
 //   buff[10] = 0x02;    
    write_modbus(&buff[0], 9);  //2 bytes (Modbus-RTU Header) + 7 bytes (Modbus-RTU Data) = 9 bytes (without CRC16)
 **/

    //Modbus-RTU Header:
    buff[0] = addr; //255 communication ID (Slave ID Address)
    buff[1] = 0x06; //function code 0x06 write to a register
    buff[2] = 0x00; //address for starting register 
    buff[3] = 0x28;
    buff[4] = 0x00; //data to be written into the register
    buff[5] = 0x02; 
    write_modbus(&buff[0], 6);  //2 bytes (Modbus-RTU Header) + 4 bytes (Modbus-RTU Data) = 6 bytes (without CRC16)


    fault_clear = 0;
    //printf("fault_clear = %d\n",fault_clear);

    frame_sent=1;
    rev_frame_size=8;
    frame_checked=0;
    //printf("x2	y2	x1	y1	x0	y0");

    current_ctime=ros::Time::now();
    if (modbus_cmd_send_delay_t_!=0)
        sleep(modbus_cmd_send_delay_t_);

}


void uwb_pos_request(void)
{
    unsigned short addr = 0x01;
    unsigned char buff[6];
    //Modbus-RTU Header:
    buff[0] = addr; //255 communication ID (Slave ID Address)
    buff[1] = 0x03; //function code 0x10 write to register(s)
    buff[2] = 0x00; //address for starting register 
    buff[3] = 0x2A;
    buff[4] = 0x00; //amount of registers used
    buff[5] = 0x04; 
    write_modbus(&buff[0], 6);  //2 bytes (Modbus-RTU Header) + 4 bytes (Modbus-RTU Data) = 6 bytes (without CRC16)

    frame_sent=1;
    rev_frame_size=13;	//double check the length of messages received
    frame_checked=0;

    current_ctime=ros::Time::now();
    if (modbus_cmd_send_delay_t_!=0)
        sleep(modbus_cmd_send_delay_t_);

}

double UWB_angle(short int x_front, short int y_front, short int x_back, short int y_back)
{
    short int delta_y = y_front - y_back;
    short int delta_x = x_front - x_back;
    return atan2(delta_y, delta_x)/PI*180;

}

int udp_server(void)
{
    server_fd = socket(AF_INET, SOCK_DGRAM, 0); //AF_INET:IPV4;SOCK_DGRAM:UDP
    if(server_fd < 0)
    {
        printf("create socket fail!\n");
        return -1;
    }

    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_addr.s_addr = htonl(INADDR_ANY); //IP地址，需要进行网络序转换，INADDR_ANY：本地地址
    ser_addr.sin_port = htons(SERVER_PORT);  //端口号，需要网络序转换

    ret_ = bind(server_fd, (struct sockaddr*)&ser_addr, sizeof(ser_addr));
    if(ret_ < 0)
    {
        printf("socket bind fail!\n");
        return -1;
    }

    return 1;
}

void handle_udp_msg(int fd_, short int x_target, short int y_target, short int x_front, short int y_front, short int x_back, short int y_back)
{
    socklen_t len;
    char buf[BUFF_LEN];  //接收缓冲区，1024字节

    memset(buf, 0, BUFF_LEN);
    len = sizeof(clent_addr);
    count_ = recvfrom(fd_, buf, BUFF_LEN, 0, (struct sockaddr*)&clent_addr, &len);  //recvfrom是拥塞函数，没有数据就一直拥塞, count_ returns the length of received msg in bytes
    if(count_ == -1)
    {
        printf("recieve data fail!\n");
        return;
    }
    printf("client:%s\n",buf);  //打印client发过来的信息
    memset(buf, 0, BUFF_LEN);

    /** at this point, fill in buf with modbus_response **/
    sprintf(buf, "%hda%hda%hda%hda%hda%hd\n", x_target, y_target, x_front, y_front, x_back, y_back);  //回复client
    //printf("sendto STAGE!!\n");
    printf("server:%s\n",buf);  //打印自己发送的信息给
    sendto(fd_, buf, BUFF_LEN, 0, (struct sockaddr*)&clent_addr, len);  //发送信息给client，注意使用了clent_addr结构体指针

}

void modbus_response(void)
{
    //Receive uart data
    unsigned char read_buffer[64];   /* Buffer to store the data received */
    int bytes_read=0;                /* Number of bytes read by the read() system call */
    int retval = 0;
    if (frame_sent) //frame transmit from Master, wait slave feedback message
    {
        //printf("read: cdt=%+3.3f, modbus_cmd_resend_time=%+3.3f\n", cdt, modbus_cmd_resend_time_);

        //printf("fileId = %d\n",fd);
        //always need to zero it first, then add our new descriptor
        FD_ZERO(&rset);     /* clear the set */
        FD_SET(fd, &rset);  /* add our file descriptor to the set */
        last_ctime=ros::Time::now();
        retval = select(fd+1, &rset, NULL, NULL, &tv);
        current_ctime=ros::Time::now();        
        cdt=(current_ctime-last_ctime).toSec();
        //printf("ctime=%+3.6f\n",cdt);
        //printf("return_select=%d\n",retval);
        if(retval > 0)
        {
            if (FD_ISSET(fd, &rset))
            {
                bytes_read = read(fd, &read_buffer, rev_frame_size); /* Read the data */
                //------------------------------------------------------
                if (bytes_read>0) //Slave Data collection and display data
                {
                    //printf("Now Receive Bytes Rxed %d\n", bytes_read); /* Print the number of bytes read */
                    for(int i=0;i<bytes_read;i++)              /* Printing only the received characters*/
                    {
                        frame_bytes[frame_idx]=read_buffer[i];
                        frame_idx++;
                        if (frame_idx>=(FRAME_BUFF_MAX-1))
                        {
                            frame_idx=0;
                            printf("Error: Frame Buff Overflow idx=%d!!\n", frame_idx);
                        }
                        //printf("0x%02X ", read_buffer[i]);
                    }

                    if(bytes_read==13) //total bytes to read
                    {
                        //read_buffer[0] slave ID
                        //read_buffer[1] function code
                        //read_buffer[2] read data length
                        tag_id = (read_buffer[3]<<8)+(read_buffer[4]); //tag id
                        pos_check = (read_buffer[5]<<8)+(read_buffer[6]); //9th bit=successful positioning, 1~8bit=successful anchor distance measuring
                        if (pos_check == 271)
                        {
                            if (tag_id == 0)
                            {
                                x_tag0 = (read_buffer[7]<<8)+(read_buffer[8]);
                                y_tag0 = (read_buffer[9]<<8)+(read_buffer[10]);
                            }
                            else if (tag_id == 1)
                            {
                                x_tag1 = (read_buffer[7]<<8)+(read_buffer[8]);
                                y_tag1 = (read_buffer[9]<<8)+(read_buffer[10]);
                            }
                            else if (tag_id == 2)
                            {
                                x_tag2 = (read_buffer[7]<<8)+(read_buffer[8]);
                                y_tag2 = (read_buffer[9]<<8)+(read_buffer[10]);
                            }
                            //z_tag0 = (read_buffer[7]<<8)+(read_buffer[8]);
                            //(read_buffer[13]<<8)+(read_buffer[14]) measure distance between tag and anchor A
                            //(read_buffer[15]<<8)+(read_buffer[16]) measure distance between tag and anchor B
                            //(read_buffer[17]<<8)+(read_buffer[18]) measure distance between tag and anchor C
                            //(read_buffer[19]<<8)+(read_buffer[20]) measure distance between tag and anchor D
                            //(read_buffer[21]<<8)+(read_buffer[22]) measure distance between tag and anchor E
                            //(read_buffer[23]<<8)+(read_buffer[24]) measure distance between tag and anchor F
                            //(read_buffer[25]<<8)+(read_buffer[26]) measure distance between tag and anchor G
                            //(read_buffer[27]<<8)+(read_buffer[28]) measure distance between tag and anchor H
                            //(read_buffer[29]<<8)+(read_buffer[30]) CRC checking
                            //printf("\nx0 = %dcm, y0 = %dcm", x_tag0, y_tag0); 
                            //printf("\nx1 = %dcm, y1 = %dcm", x_tag1, y_tag1);
                            //angle_AGV = UWB_angle(x_tag1, y_tag1, x_tag0, y_tag0);
                            //printf("\n%d	%d	%d	%d	%d	%d", x_tag2, y_tag2, x_tag1, y_tag1, x_tag0, y_tag0); 
                            handle_udp_msg(server_fd, x_tag2, y_tag2, x_tag1, y_tag1, x_tag0, y_tag0);                    
                        }                    
                    }

                    last_ctime=ros::Time::now();
                    cdt=(last_ctime-current_ctime).toSec();
                    //printf("frame_idx=%d, ctime=%+3.6f\n", frame_idx, cdt);
                }
            }
        }
        else //Receiver Timeout
        {
            frame_sent=0;
            frame_resend_cnt++;
            //printf("Warning: Receiver Time-out:%+3.3f, Now Starting Resend (frame_resend_cnt=%d, trans_step=%d)\n", cdt, frame_resend_cnt, trans_step);

            //Re-send command count----------------------------------
            if (modbus_cmd_resend_max_!=0) //modbus_cmd_resend_max_=0 is not appcation
            {
                if (frame_resend_cnt>=modbus_cmd_resend_max_)
                {
                    frame_resend_cnt=0;
                    printf("Error: Modbus Recveie Error, Please check your connect!!\n");
                }
            }
            else
                frame_resend_cnt=0;
        }

            //------------------------------------------------------
        if (frame_idx>=rev_frame_size) //Frame received then crc16 check
        {
            unsigned short crc_num;
            unsigned char crc_data[2];
            frame_recved=1;
            frame_idx=0;
            frame_crc16=crc_fn(&frame_bytes[0], rev_frame_size-2); //modbus CRC16
            crc_data[1]=(unsigned char)(frame_crc16 & 0xff);
            crc_data[0]=(unsigned char)((frame_crc16 >> 8) & 0xff);
            frame_crc16=((crc_data[1] << 8) &0xFF00) | (crc_data[0] & 0xFF);
            crc_num= ((frame_bytes[rev_frame_size-2] << 8) &0xFF00) | (frame_bytes[rev_frame_size-1] & 0xFF);

            //printf("frame: 0x");
            //for (int i=0;i<rev_frame_size;i++)
            //    printf("%02X ", frame_bytes[i]);
            //printf("\n");
            //printf("frame_crc16=%d(0x%02X%02X), crc_num=%d(0x%02X%02X)\n", frame_crc16, crc_data[1], crc_data[0], crc_num, frame_bytes[rev_frame_size-2], frame_bytes[rev_frame_size-1]);
            if (frame_crc16==crc_num)
            {
                frame_checked=1;
                //printf("CRC16 Check Pass.\n"); //Impant.
            }
            else
            {
                frame_sent=0;
                frame_resend_cnt++;
                printf("Error: CRC16 Check Fail, Now Starting Resend (frame_resend_cnt=%d, trans_step=%d) Command to Slave!!.\n", frame_resend_cnt, trans_step);

                //printf("I am here\n");

                //Re-send command count----------------------------------
                if (modbus_cmd_resend_max_!=0) //modbus_cmd_resend_max_=0 is not appcation
                {
                    if (frame_resend_cnt>=modbus_cmd_resend_max_)
                    {
                        frame_resend_cnt=0;
                        printf("Error: Modbus Recveie Error, Please check your connect!!\n");
                    }
                }
                else
                    frame_resend_cnt=0;
            }
        }

            //------------------------------------------------------
        if (frame_checked) //Modbus receive and CRC16 check OK!!
        {
            last_ctime=ros::Time::now();
            cdt=(last_ctime-current_ctime).toSec();
            //printf("OK: trans_step=%d, frame_checked:ctime=%+3.3f\n",trans_step, cdt);

            frame_resend_cnt=0;
            trans_step = 0;
            //trans_step=trans_next;
            frame_sent=0;
            cdt=0;
        }
    }  
}

void uart_setting(void)
{
    //Std::String to char* (string)
    char *cstr = new char[modbus_port_.length() + 1];
    strcpy(cstr, modbus_port_.c_str());    

    fd = open(cstr, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        perror("open_port: Unable to open Modbus Port - ");
        exit(-1);
    }
    else
    fcntl(fd, F_SETFL, 1);

    delete [] cstr; //delete temp char* (string)

    struct termios options;
    
    speed_t baud = B115200;
    if (modbus_baud_=="115200")
    	baud = B115200;
    else if (modbus_baud_=="57600")
	baud = B57600;
    else if (modbus_baud_=="38400")
	baud = B38400;
    else if (modbus_baud_=="19200")
	baud = B19200;
    else if (modbus_baud_=="9600")
	baud = B9600;

    tcgetattr(fd, &options); /* Get the original setting */
    cfsetispeed(&options, baud); /* Setting the baudrate */
    cfsetospeed(&options, baud);
    options.c_cflag |= (CLOCAL|CREAD); /* enable the receiver and set local mode */
    options.c_cflag &= ~CSIZE; /* setting the character size (8-bits) */
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB; /* Setting Parity Checking: NONE (NO Parity) */
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS; /* Disable Hardware flow control */
    options.c_iflag &= ~INPCK; /* Disable Input Parity Checking */
    options.c_iflag &= ~(IXON|IXOFF|IXANY); /* Disable software flow control */
    options.c_iflag &= ~(IGNPAR|ICRNL);
    options.c_oflag &= ~OPOST; /* output raw data */
    options.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG); /* disablestd input */
    tcflush(fd, TCIFLUSH); /* clean the current setting */
    tcsetattr(fd, TCSANOW, &options); /* Enable the new setting right now */

    //Timeout Receiver Setting
    tv.tv_sec = modbus_recveie_timeout_; //unit: sec
    tv.tv_usec = 0;                      //unit: usec
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agv_modbus_link_node");
    ros::NodeHandle n;
    
    //----------------------------------------------------------------
    n.getParam("/comm_link/main_freq", main_freq_) ;
    n.getParam("/comm_link/modbus_cmd_send_delay_t", modbus_cmd_send_delay_t_);
    n.param("/comm_link/modbus_port", modbus_port_, std::string("/dev/ttyUSB0"));   //device connection in Linux OS
    n.param("/comm_link/modbus_baud", modbus_baud_, std::string("115200"));         //device baud rate
    n.getParam("/comm_link/modbus_cmd_resend_max", modbus_cmd_resend_max_);
    n.getParam("/comm_link/modbus_cmd_resend_time", modbus_cmd_resend_time_);
    n.getParam("/comm_link/modbus_recveie_timeout", modbus_recveie_timeout_);

    ros::Rate loop_rate(main_freq_);

    get_new_cmd = 1;
    uart_setting();
    udp_init = udp_server();  // UDP server socket(), bind()
    if (udp_init != 1)
    {
        printf("UDP initialization failure!!\n");
        return -1;
    }

    uwb_pos_cmd();  //send start positioning command   
    modbus_response();
    
    while (ros::ok())
    {
        if (get_new_cmd == 1)
        {
            uwb_pos_request();  		//request for positioning response
            get_new_cmd = 2;
        }
        else if (get_new_cmd == 2)
        {
            modbus_response();  
            get_new_cmd = 1;
        }

        //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        ros::spinOnce();
        loop_rate.sleep();
    }

    close(fd);
    close(server_fd);   // UDP server off
    return 0;
}

