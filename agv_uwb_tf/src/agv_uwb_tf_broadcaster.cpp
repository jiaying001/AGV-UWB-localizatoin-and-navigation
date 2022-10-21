#include <ros/ros.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <signal.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#define MAIN_FREQ   400.0
#define SERVER_PORT 8888
#define SENDBUFF_LEN 128
#define RECVBUFF_LEN 2048
#define SERVER_IP "192.168.0.107"
#define DIFF_MAP_X 1.2 
#define DIFF_MAP_Y 1.2

#define PI 3.141593

//ROS Parameter
double main_freq_=MAIN_FREQ; 

//------------------
//static int get_new_cmd = 0;

//UDP client
int udp_client_init = -1;
int client_fd;
struct sockaddr_in ser_addr;
char sendbuf[SENDBUFF_LEN] = "REQ UWB POS!\n";

double diff_map_x = DIFF_MAP_X; 
double diff_map_y = DIFF_MAP_Y;
short int x_tag0 = 0;
short int y_tag0 = 0;
short int x_tag1 = 0;
short int y_tag1 = 0;
short int x_tag2 = 0;
short int y_tag2 = 0;
double angle_agv = 0.0; 
double angle_target = 0.0;

void TARposecallback(const short int x_tag2, const short int y_tag2, const double angle)		
{
    	static tf2_ros::TransformBroadcaster br;		//"transform" publisher
	geometry_msgs::TransformStamped transformStamped;	//type of publisher
   
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "map";
	transformStamped.child_frame_id = "uwb_target";
	transformStamped.transform.translation.x = x_tag2/100.0 - diff_map_x;	//input UWB messages
	transformStamped.transform.translation.y = y_tag2/100.0 - diff_map_y;	//input UWB messages
	transformStamped.transform.translation.z = 0.0;
	tf2::Quaternion q;
    	q.setRPY(0, 0, angle);					//input UWB angle messages
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);			//~.publish()
}

void AGVposecallback(const short int x_tag1, const short int y_tag1, const double angle)		
{
    	static tf2_ros::TransformBroadcaster br;		//"transform" publisher
	geometry_msgs::TransformStamped transformStamped;	//type of publisher
   
	transformStamped.header.stamp = ros::Time::now();
	//transformStamped.header.frame_id = "odom";		//for gmapping
	//transformStamped.child_frame_id = "base_link";	//for gmapping
	transformStamped.header.frame_id = "map";
	transformStamped.child_frame_id = "uwb_agv";
	//---for error computation, the coordinate of midpoints between tag1 and tag0 is taken---//
	transformStamped.transform.translation.x = (x_tag0 + x_tag1)/200.0 - diff_map_x;	//input UWB messages
	transformStamped.transform.translation.y = (y_tag0 + y_tag1)/200.0 - diff_map_y;	//input UWB messages
	transformStamped.transform.translation.z = 0.0;
	tf2::Quaternion q;
    	q.setRPY(0, 0, angle);					//input UWB angle messages
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);			//~.publish()
}

double UWB_angle(short int x_front, short int y_front, short int x_back, short int y_back)
{
    short int delta_x = x_front - x_back; 	
    short int delta_y = y_front - y_back; 
    return atan2(delta_y, delta_x); 
}


void alarm_handler(int sigon)
{
    socklen_t len;
    len = sizeof(ser_addr);
    printf("alarm interruption!!\n");
    sendto(client_fd, sendbuf, SENDBUFF_LEN, 0, (struct sockaddr*)&ser_addr, len);
    return;
}

void udp_msg_sender_loop(int fd, struct sockaddr* dst)
{
    socklen_t len;
    struct sockaddr_in src;
    char recvbuf[RECVBUFF_LEN] = {0};

    //SINGALRM function to resend sendto() to UWB server
    struct sigaction alr;
    alr.sa_handler = alarm_handler;
    alr.sa_flags = SA_NOMASK;
    alr.sa_restorer = NULL;

    while(ros::ok())
    {
	len = sizeof(*dst);
	printf("client:%s\n",sendbuf);  //打印自己发送的信息
	sendto(fd, sendbuf, SENDBUFF_LEN, 0, dst, len);

	alarm(1);  //1sec alarm
	sigaction(SIGALRM,&alr,NULL);    

	recvfrom(fd, recvbuf, RECVBUFF_LEN, 0, (struct sockaddr*)&src, &len);  //接收来自server的信息
	//---------------------------------
	printf("server:%s\n",recvbuf);
	sscanf (recvbuf, "%hda%hda%hda%hda%hda%hd", &x_tag2, &y_tag2, &x_tag1, &y_tag1, &x_tag0, &y_tag0);
	angle_agv = UWB_angle(x_tag1, y_tag1, x_tag0, y_tag0);
	angle_target = UWB_angle(x_tag2, y_tag2, x_tag0, y_tag0);
	printf("x2: %d, y2: %d, angle_target: %.2f\nx1: %d, y1: %d, angle_agv: %.2f\nx0: %d, y0: %d\n", x_tag2, y_tag2, angle_target, x_tag1, y_tag1, angle_agv, x_tag0, y_tag0);
	AGVposecallback(x_tag1, y_tag1, angle_agv);
	TARposecallback(x_tag2, y_tag2, angle_target);
	memset(recvbuf, 0, RECVBUFF_LEN);
	//---------------------------------
	alarm(0);  //close alarm
    }
}


int udp_client_socket()
{
    client_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(client_fd < 0)
    {
        printf("create socket fail!\n");
        return -1;
    }

    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    //ser_addr.sin_addr.s_addr = htonl(INADDR_ANY);  //注意网络序转换
    ser_addr.sin_port = htons(SERVER_PORT);  //注意网络序转换

    return 1;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "uwb_tf2_broadcaster");		//must be initialized to publish rostopic
    //ros::NodeHandle n;
   
    udp_client_init = udp_client_socket();
    if (udp_client_init == -1)
    {
        printf("UDP client initialization error!!\n");
        return -1;
    }

    udp_msg_sender_loop(client_fd, (struct sockaddr*)&ser_addr);

    close(client_fd);

    //ros::spin();
    return 0;
}
