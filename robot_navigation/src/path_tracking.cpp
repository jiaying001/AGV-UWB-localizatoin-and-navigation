//---Potential Field Method---//
//---try to separate navigation method of [PFM to detect obs] only and [straight_line-2 to move towards endpoint]---//

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Odometry.h"
#include "comm.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <sstream>

#define MAIN_FREQ 30.0
#define REPULSIVE_COEFF 0.7
#define ATTRACTIVE_COEFF 1.0
#define LASER_SCAN_DIST 1.2
#define LASER_SCAN_RESOLUTION 1080

ros::Publisher vel_pub; 
ros::Subscriber scan_sub; 

geometry_msgs::Twist vel_pub_msg;

double main_freq = MAIN_FREQ;
int timer = 0; 

//---Parameters for Potential Field Method using LIDAR---//
double repul_coeff = REPULSIVE_COEFF;
double att_coeff = ATTRACTIVE_COEFF; 
double scan_dist = LASER_SCAN_DIST; 
int scan_res = LASER_SCAN_RESOLUTION; 

//---Variables for tag following---//
double x_tgt_last, y_tgt_last; 
double x, y, z, angle; 
double x_tgt, y_tgt, z_tgt, angle_tgt; 
double x_otb, y_otb, z_otb, angle_otb; 
bool tf_flag, tf_flag_tgt, tf_flag_otb; 
double x_uwb_last, y_uwb_last, angle_uwb_last; 
double x_otb_last, y_otb_last, angle_otb_last; 
double resultant_Angle; 
bool tag_shift, vel_cmd_init_flag; 

//---Variables for path tracking---//
double lastPose[5000][2] = {0}; 
int j = 0; 
int k = 0; 


void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  int i; 
  double angleObs; 
  double repul_x = 0; 
  double repul_y = 0; 
  double repul_Amp = 0;
  double repul_Angle = 0;
  double att_Amp = 0; 
  double att_Angle = 0; 
  double resultant_x = 0; 
  double resultant_y = 0; 
  
  resultant_Angle = 0; 

  //---This Hokuyo Lidar model provides max. 270deg with 1080 step size---// 
  //---By limiting the steps from 180th to 900th, only 180deg in front of the AGV is taken into consideration---//
  for (i=180; i<900; i++)
  {
    if (scan_msg->ranges[i] <= scan_dist)
    {
      angleObs = (1.5*PI/(scan_res-1))*i - (0.25*PI) + angle; 
      repul_x -= scan_msg->ranges[i] * cos(angleObs); 
      repul_y -= scan_msg->ranges[i] * sin(angleObs); 
      repul_Amp += 0.5*repul_coeff*pow(((1.0/scan_msg->ranges[i])-(1.0/scan_dist)),2.0); 
    }
  }

  //printf("%.2f, %.2f\n", repul_x, repul_y); 
  repul_Angle = atan2(repul_y, repul_x);   
  //printf("Repul Force: %.2f, %.2f\n", repul_Amp, repul_Angle); 

  att_Amp = 0.5*att_coeff*(pow((x_tgt_last - x),2.0)+pow((y_tgt_last - y),2.0)); 
  att_Angle = atan2((y_tgt_last - y), (x_tgt_last - x)); 
  //printf("Att Force: %.2f, %.2f\n", att_Amp, att_Angle); 

  if (att_Amp > 0.40) 		//not using repul_Angle here because obstacles may exist near the target point
  {
    resultant_x = repul_Amp*cos(repul_Angle) + att_Amp*cos(att_Angle); 
    resultant_y = repul_Amp*sin(repul_Angle) + att_Amp*sin(att_Angle); 
    printf("Resultant Angle: %.2f\n", atan2(resultant_y, resultant_x)); 
    resultant_Angle = atan2(resultant_y, resultant_x); 
  }
  else 
    resultant_Angle = 9999; 
} 

void vel_cmd()
{
  double x_diff, y_diff, angle_trans, angle_diff, dist; 
  
  if (vel_cmd_init_flag == 0) 
  {
    x_uwb_last = x; 
    y_uwb_last = y; 
    angle_uwb_last = angle; 
    vel_cmd_init_flag = 1; 
  }
/**
  else if (int(round(dt)) % 2 == 0 && vel_cmd_init_flag == 1)
  {
    printf("Update UWB and ODOM!!\n"); 
    x_uwb_last = x; 
    y_uwb_last = y; 
    angle_uwb_last = angle; 
    x_otb_last = x_otb; 
    y_otb_last = y_otb; 
    angle_otb_last = angle_otb; 
  }
**/
  else if (((fabs(x_otb - x_otb_last) > 0.5) || (fabs(y_otb - y_otb_last) > 0.5) || (fabs(angle_otb - angle_otb_last) > PI/2) )&& vel_cmd_init_flag == 1)
  {
    printf("Update UWB and ODOM!!\n"); 
    x_uwb_last = x; 
    y_uwb_last = y; 
    angle_uwb_last = angle; 
    x_otb_last = x_otb; 
    y_otb_last = y_otb; 
    angle_otb_last = angle_otb; 
  }


  x_diff = x_tgt_last - (x_uwb_last - x_otb_last + x_otb); 
  y_diff = y_tgt_last - (y_uwb_last - y_otb_last + y_otb); 
  dist = sqrt(pow(x_diff,2)+pow(y_diff,2)); 
  angle_trans = angle_uwb_last - angle_otb_last + angle_otb; 
  angle_diff = atan2(sin(angle_trans - resultant_Angle), cos(angle_trans - resultant_Angle)); 

  if (resultant_Angle < 9999)
  {
    if (angle_diff > 0.10 && dist > 0.30)
    {
      //printf("ACrotate!!\n");
      //printf("%.2f, %.2f\n", angle, resultant_Angle);
      vel_pub_msg.linear.x = 0.02;
      vel_pub_msg.angular.z = -0.15;
      vel_pub.publish(vel_pub_msg);
    }
    else if (angle_diff < -0.10 && dist > 0.30)
    {
      //printf("Crotate!!\n");
      //printf("%.2f, %.2f\n", angle, resultant_Angle);
      vel_pub_msg.linear.x = 0.02;
      vel_pub_msg.angular.z = 0.15;
      vel_pub.publish(vel_pub_msg);
    }
    else if (dist > 0.10)
    {
      //printf("Straight!!\n");
      vel_pub_msg.linear.x = 0.20; 		//change the velocity for different dist
      vel_pub.publish(vel_pub_msg);
    }
  }
  else
  {
    printf("Reached point %d\n", k);
    vel_pub_msg.angular.z = 0.0;
    vel_pub_msg.linear.x = 0.0; 		//change the velocity for different dist
    vel_pub.publish(vel_pub_msg); 
    vel_cmd_init_flag == 0; 
    k++; 
  }
}

double target_pos_shift()
{  
  double dx = 0; 
  double dy = 0; 
  if (lastPose[j][0] != -999 && lastPose[j][1] != -999) 
  {
    dx = fabs(x_tgt - lastPose[j][0]); 
    dy = fabs(y_tgt - lastPose[j][1]); 
  }
  if ((dx > 0.20)||(dy >0.20))
  {
    printf("%.2f, %.2f\n", dx, dy);
    j++; 
    lastPose[j][0] = x_tgt; 
    lastPose[j][1] = y_tgt; 
    return 1; 
  }
  else
    return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_tracking");
  ros::NodeHandle n;
  vel_pub = n.advertise<geometry_msgs::Twist>("/Pioneer3AT/cmd_vel", 1000); 
  scan_sub = n.subscribe("/scan", 1, scanCallback); 
  ros::Rate loop_rate(main_freq);

  GetTFPose tf1(2, main_freq); //tf_rev_delay_t, programme_freq  
  GetTFPose tf2(2, main_freq); 
  GetTFPose tf3(2, main_freq); 

  memset(lastPose, -999, sizeof(lastPose)); 

  while (!tf_flag_tgt)
    tf2.GetPose("map","uwb_target", &x_tgt_last, &y_tgt_last, &z_tgt, &angle_tgt, &tf_flag_tgt); 

  lastPose[j][0] = x_tgt_last; 
  lastPose[j][1] = y_tgt_last;
  printf("%.2f, %.2f, %d\n", x_tgt_last, y_tgt_last, j);

  while (ros::ok())
  { 
    tf1.GetPose("map","uwb_agv", &x, &y, &z, &angle, &tf_flag);
    tf2.GetPose("map","uwb_target", &x_tgt, &y_tgt, &z_tgt, &angle_tgt, &tf_flag_tgt); 
    tf3.GetPose("odom", "base_link", &x_otb, &y_otb, &z_otb, &angle_otb, &tf_flag_otb);  

    if (tf_flag && tf_flag_tgt && tf_flag_otb)
    {
      tag_shift = target_pos_shift();  //AGV will stop when the UWB targeting tag is shifted
      if (tag_shift) 
      {
        printf("waiting tag to stop\n");    
        vel_pub_msg.angular.z = 0.0;
        vel_pub_msg.linear.x = 0.0; 
        vel_pub.publish(vel_pub_msg);    
        printf("%.2f, %.2f\n", lastPose[j][0], lastPose[j][1]);
        timer = 0;    
      }
      else
      { 
        if (timer >= 90)
        {
          if (k <= j)
          {
            printf("k = %d\n", k);
            printf("following tag\n"); 
            x_tgt_last = lastPose[k][0]; 
            y_tgt_last = lastPose[k][1];  
            vel_cmd(); 
          }
        } 
        else 
          timer++; 
      }
    }
 
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

