/*
//Title: Common funcation (IoT Lab)
//Author: Chen, Chun-Lin
//Data: 2015/06/17
//Update: 2016/03/31
*/

#ifndef COMM_H
#define COMM_H

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
using namespace std;
#define PI                      3.1415926
#define DTR                     PI/180.0        //Degree to Rad
#define RTD                     180.0/PI        //Rad to Degree

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

//PID controller
double PID_CTRL(double e, double *eold, double *ki_old, double kp, double ki, double kd, double ts)
{
    double kp_tm, ki_tm, kd_tm;
    double u;

    kp_tm=kp*e;
    ki_tm=ki*e*ts+(*ki_old);
    *ki_old=ki_tm;
    kd_tm=kd*(e-(*eold))/ts;
    *eold=e;
    u=kp_tm+ki_tm+kd_tm;

    return u;
}

class GetTFPose
{
  public:
    GetTFPose();
    GetTFPose(double start_delay_time, double freq);
    ~GetTFPose();
    void GetPose(std::string source_frame, std::string target_frame, double *out_x, double *out_y, double *out_z, double *out_yaw, bool *out_flag);
    void GetPose2(std::string source_frame, std::string target_frame, double *out_x, double *out_y, double *out_z, double *out_pitch, bool *out_flag);
    void init(double start_delay_time, double freq);

    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    double now_time;
    bool rev_flag;

  private:
    tf::TransformListener tf_;
    bool first_delay_flag;
    int rev_delay;
    int rev_delay_cnt;
    bool debug_flag;
};

GetTFPose::GetTFPose()
{
    init(5.0, 10.0);
}

GetTFPose::GetTFPose(double start_delay_time, double freq)
{
    init(start_delay_time, freq);
}

GetTFPose::~GetTFPose()
{
}

void GetTFPose::init(double start_delay_time, double freq)
{
    rev_delay=(int)(start_delay_time*freq);
    rev_delay_cnt=0;
    first_delay_flag=0;
}

void GetTFPose::GetPose(std::string source_frame, std::string target_frame, double *out_x, double *out_y, double *out_z, double *out_yaw, bool *out_flag)
{
    //Get Pose from TF listener
    //printf("first_delay_flag=%d, rev_delay_cnt=%d rev_delay=%d\n", first_delay_flag, rev_delay_cnt, rev_delay);
    if (first_delay_flag)
    {
        tf::StampedTransform rev_tf;
        rev_flag=0;
        try{
            tf_.lookupTransform(source_frame, target_frame, ros::Time(0), rev_tf);
            x=rev_tf.getOrigin().x();
            y=rev_tf.getOrigin().y();
            z=rev_tf.getOrigin().z();
            yaw=tf::getYaw(rev_tf.getRotation());
            now_time=ros::Time::now().toSec();
            rev_flag=1;
            *out_x=x;
            *out_y=y;
            *out_z=z;
            *out_yaw=yaw;
            *out_flag=rev_flag;
        }
        catch (tf::TransformException ex){
            if (debug_flag)
                ROS_ERROR("%s",ex.what());
        }
    }
    else
    {
        rev_delay_cnt++;
        if (rev_delay_cnt>=rev_delay)
        {
            rev_delay_cnt=rev_delay;
            first_delay_flag=1;
        }
    }
}

void GetTFPose::GetPose2(std::string source_frame, std::string target_frame, double *out_x, double *out_y, double *out_z, double *out_pitch, bool *out_flag)
{
    //Get Pose from TF listener
    //printf("first_delay_flag=%d, rev_delay_cnt=%d rev_delay=%d\n", first_delay_flag, rev_delay_cnt, rev_delay);
    if (first_delay_flag)
    {
        tf::StampedTransform rev_tf;
        rev_flag=0;
        try{
            tf_.lookupTransform(source_frame, target_frame, ros::Time(0), rev_tf);
            x=rev_tf.getOrigin().x();
            y=rev_tf.getOrigin().y();
            z=rev_tf.getOrigin().z();	    
	    tf::Quaternion q;
     	    q = rev_tf.getRotation();
     	    tf::Matrix3x3 m(q);
            m.getRPY(roll,pitch,yaw);
            now_time=ros::Time::now().toSec();
            rev_flag=1;
            *out_x=x;
            *out_y=y;
            *out_z=z;
            *out_pitch=pitch;
            *out_flag=rev_flag;
        }
        catch (tf::TransformException ex){
            if (debug_flag)
                ROS_ERROR("%s",ex.what());
        }
    }
    else
    {
        rev_delay_cnt++;
        if (rev_delay_cnt>=rev_delay)
        {
            rev_delay_cnt=rev_delay;
            first_delay_flag=1;
        }
    }
}

class StatesMachine
{
  public:
    StatesMachine();
    ~StatesMachine();
    void Reset_State_Machine(void);
    void State_Flag_Update(void);

    //enum STATE {STATE_STANDBY, STATE_SETTING, STATE_DELAY1, STATE_YAW, STATE_DELAY2, STATE_OBS_SENSING, STATE_MOVE, STATE_DELAY3, STATE_SAVE, STATE_INC_ACTP};
    //enum STATE {STATE_STANDBY, STATE_WAYP, STATE_INC_WAYP, STATE_SETP, STATE_INC_PLAN, STATE_SENSING};
    enum STATE {S0, S1, S2, S3, S4, S5, S6, S7, S8, S9};
    /* State flags are declared here. */
    STATE state_before_change;
    STATE state_prev;
    STATE state;
    bool state_changing;

  private:
};

StatesMachine::StatesMachine()
{
    Reset_State_Machine();
}

StatesMachine::~StatesMachine()
{
}

void StatesMachine::Reset_State_Machine(void)
{
    state_before_change = S0;
    state_prev = S0;
    state = S0;
    state_changing = false;
}

/* Conditions and flags for state transition. */
void StatesMachine::State_Flag_Update(void)
{
    state_changing = (state != state_prev);
    state_before_change = (state_changing) ? state_prev : state_before_change;
    state_prev = state;
}

typedef struct
{
    double roll;
    double pitch;
    double yaw;
} Pose;

typedef struct
{
    double x;
    double y;
    double z;
    double a;
    double rev_flag;
} Posa;

//Euler angle to Quaternion
geometry_msgs::Quaternion Euler_to_Quat(double roll, double pitch, double yaw)
{
    tf::Quaternion tf_quat;
    geometry_msgs::Quaternion msg_quat;
    tf_quat=tf::createQuaternionFromRPY(roll, pitch, yaw);
    tf::quaternionTFToMsg(tf_quat, msg_quat);
    return msg_quat;
}

//Quaternion to one Euler angle
double Quat_to_Euler(geometry_msgs::Quaternion msg_quat, int sel)
{
    tf::Quaternion tf_quat;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(msg_quat, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    if (sel==0)
        return roll;
    else if (sel==1)
        return pitch;
    else if (sel==2)
        return yaw;
}

//Quaternion to All Euler angle
Pose Quat_to_AllEuler(geometry_msgs::Quaternion msg_quat)
{
    tf::Quaternion tf_quat;
    Pose pose;
    tf::quaternionMsgToTF(msg_quat, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(pose.roll, pose.pitch, pose.yaw);
    return pose;
}

//Quaternion to All Euler angle (Input TF format)
Pose Quat_to_AllEuler_TF(tf::Quaternion tf_quat)
{
    Pose pose;
    tf::Matrix3x3(tf_quat).getRPY(pose.roll, pose.pitch, pose.yaw);
    return pose;
}

//Quaternion to Yaw
double Quat_to_Yaw(geometry_msgs::Quaternion msg_quat)
{
    double yaw;
    //TF Function Converte to Yaw Angle (Euler Angle)
    geometry_msgs::Pose msg_pose;
    msg_pose.orientation=msg_quat;
    tf::Pose tf_pose;
    tf::poseMsgToTF(msg_pose, tf_pose);
    yaw = tf::getYaw(tf_pose.getRotation());
    return yaw;
}

//Calculate Sign
int SIG_F(double num)
{
    if (num>=0)
        return 1;
    else
        return -1;
}

double SIG(double num)
{
    if (num>=0)
        return 1.0;
    else
        return -1.0;
}

//Error compare
bool ECP(double x, double p, double bias)
{
    if (fabs(x-p) > (bias) )
        return 1;
    else
        return 0;
}

double MIN_D(double a, double b)
{
	if (a<b)
		return a;
	else
		return b;	
}

//Calculate Distance in XY plane
double DIS_XY(double pa_x, double pa_y, double pb_x, double pb_y)
{
    double distance;
    distance=pow(pow(pa_x-pb_x,2)+pow(pa_y-pb_y,2),0.5);
    return distance;
}

//Calculate Distance in XYZ plane
double DIS_XYZ(double pa_x, double pa_y, double pa_z, double pb_x, double pb_y, double pb_z)
{
    double distance;
    distance=pow(pow(pa_x-pb_x,2)+pow(pa_y-pb_y,2)+pow(pa_z-pb_z,2),0.5);
    return distance;
}

//Calculate Angle in XY plane (Results: -pi ~ 0 ~ pi)
double ANG_XY(double pa_x, double pa_y, double pb_x, double pb_y)
{
    double angle;
    angle=atan2( pa_y-pb_y, pa_x-pb_x );
    return angle;
}

//Calculate Angle in XYZ plane (Results: -pi ~ 0 ~ pi)
double ANG_XYZ(double pa_x, double pa_y, double pa_z, double pb_x, double pb_y, double pb_z)
{
    double angle;
    double distance;
    distance=pow(pow(pa_x-pb_x,2)+pow(pa_y-pb_y,2),0.5);
    angle=atan2( pa_z-pb_z, distance);
    return angle;
}

//Calculate Minmun Rotation angle
double MIN_ROT_ERR(double target_a, double now_a)
{
    double err_a;
    err_a=target_a-now_a;
    if (fabs(err_a)>PI)
    {
        if (target_a>0)
            target_a=target_a-2*PI;
        else
            target_a=target_a+2*PI;
        err_a=target_a-now_a;
    }
    return err_a;
}

//Rotation direction select (rdir_sel= -1 : clockwise (CW), tdir_sel= 1 : counterclockwise (CCW) )
double FIX_ROT_ERR(double target_a, double now_a, int rdir_sel)
{
    int err_f;
    double err_a;

    err_a=target_a-now_a;
    err_f=SIG_F(err_a);
    if (err_f!=rdir_sel)
    {
        if (rdir_sel>=0)
            err_a=(target_a+2*PI)-now_a;
        else
            err_a=(target_a-2*PI)-now_a;
    }
    return err_a;
}

//Calculate Angle Err
double ANG_XY_ERR(double pa1_x, double pa1_y, double pb1_x, double pb1_y, double pa2_x, double pa2_y, double pb2_x, double pb2_y)
{
    double angle1, angle2;
    double angle_err;

    angle1=atan2(pa1_y-pb1_y, pa1_x-pb1_x );
    angle1=angle1*RTD;

    angle2=atan2(pa2_y-pb2_y, pa2_x-pb2_x );
    angle2=angle2*RTD;

    angle_err=angle1-angle2;

    if (fabs(angle_err)>180)
    {
        if (angle1>0)
            angle1=angle1-2*180;
        else
            angle1=angle1+2*180;
        angle_err=angle1-angle2;
    }

    return angle_err;
}

//Reaminder (Similar mod in MATLAB)
double FMOD(double dividend, double divisor)
{
    double remainder;
    double cf;
    int c;

    c=dividend/divisor;
    cf=dividend/divisor;
    if (cf<0)
	c=c-1;
    remainder=dividend-(divisor*c);

    return remainder;
}

double INPI(double value)
{
    return FMOD(value+PI, 2*PI)-PI;
}

int Check_XY_Arrive(Posa pa, Posa pb, double pos_xerr_bias, double pos_yerr_bias)
{
    int flag=0;
    if ( (fabs(pa.x-pb.x)<=pos_xerr_bias) && (fabs(pa.y-pb.y)<=pos_yerr_bias) )
        flag=1;
    return flag;
}

int Check_Z_Arrive(Posa pa, Posa pb, double pos_zerr_bias)
{
    int flag=0;
    if ( (fabs(pa.z-pb.z)<=pos_zerr_bias) )
        flag=1;
    return flag;
}

int Check_A_Arrive(Posa pa, Posa pb, double pos_aerr_bias)
{
    int flag=0;
    if ( (fabs(pa.a-pb.a)<=(pos_aerr_bias*3.1415926/180.0)) )
        flag=1;
    return flag;
}

int Check_XY_DIS_Arrive(Posa pa, Posa pb, double dis_xyerr_bias)
{
    double dis;
    int flag=0;
    dis=DIS_XY(pa.x, pa.y, pb.x, pb.y);
    if (dis<=dis_xyerr_bias)
        flag=1;
    return flag;
}

int Check_XYZ_DIS_Arrive(Posa pa, Posa pb, double dis_xyzerr_bias)
{
    double dis;
    int flag=0;
    dis=DIS_XYZ(pa.x, pa.y, pa.z, pb.x, pb.y, pb.z);
    if (dis<=dis_xyzerr_bias)
        flag=1;
    return flag;
}

void Sat_Value_uc(unsigned char *In, unsigned char Upp, unsigned char Low)
{
    unsigned char data;
    data=*In;
    if (data>Upp) data=Upp;
    if (data<Low) data=Low;
    *In=data;
}

void Sat_Value_ui(unsigned int *In, unsigned int Upp, unsigned int Low)
{
    unsigned int data;
    data=*In;
    if (data>Upp) data=Upp;
    if (data<Low) data=Low;
    *In=data;
}

void Sat_Value(double *In, double Upp, double Low)
{
    unsigned int data;
    data=*In;
    if (data>Upp) data=Upp;
    if (data<Low) data=Low;
    *In=data;
}
#endif // COMM_H
