#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Char.h>
#include "tt15_linux_serial.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <fstream>
#include <chrono>

class TT15Driver
{
public:
    TT15Driver();
    ~TT15Driver();
    double odom_x, odom_y, odom_theta, odom_v, odom_omega;
    double set_v, set_omega;
    double set_u1,set_u2;
    nav_msgs::Odometry odom_msg;
    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point time_current = std::chrono::steady_clock::now();
    inline void updateSeq() { seq_counter_++; }
    void publishMsg();
    bool updateOdom();
private:
    enum Mode{
        CMDVEL_CONTROL,
        U1U2_CONTROL
    };
    Mode mode;

    bool enable_flag_;
    const int enable_button_;
    ros::NodeHandle nh_;
    int buttonX_, buttonY_, axesFork_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber enable_sub_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster odom_broadcaster_;
    unsigned int seq_counter_;

    std::string port;
    TT15SerialPort* tt15_sp;

    void cmdVelCallBack(const geometry_msgs::Twist& msg);
    void joyCallBack(const sensor_msgs::Joy::ConstPtr& Joy);
    void forkCtrlCallBack(const std_msgs::Char& msg);
};

TT15Driver::~TT15Driver()
{
    if(tt15_sp)
        delete tt15_sp;
}

TT15Driver::TT15Driver():
    odom_x(0),
    odom_y(0),
    odom_theta(0),
    odom_v(0),
    odom_omega(0),
    buttonX_(0),
    buttonY_(3),
    axesFork_(5),
    enable_flag_(false),
    set_u1(0),set_u2(0),set_v(0),set_omega(0),
    enable_button_(4),
    seq_counter_(0),
    mode(CMDVEL_CONTROL),
    port("ttyUSB0")
{
    nh_.param("port_name", port, port);
    tt15_sp = new TT15SerialPort("/dev/"+port);
    cmd_vel_sub_ = nh_.subscribe("/tt15_cmd_vel", 1, &TT15Driver::cmdVelCallBack, this);
    enable_sub_ = nh_.subscribe("/joy", 10, &TT15Driver::joyCallBack, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/encoder_odom", 1000, true);

}



void TT15Driver::cmdVelCallBack(const geometry_msgs::Twist& msg)
{
    // ROS_INFO("call back cmd");
    if(enable_flag_)
    {
        set_v = msg.linear.x;
        set_omega = msg.angular.z;
        // set_u1 = 5;
        // set_u2 = -10;
        ROS_INFO("setpoint: linear velocity: %.4lf m/s, angular velocity: %.4lf rad/s", set_v, set_omega);
    }
    else
    {
        set_v = 0;
        set_omega = 0;
        set_u1 = 0;
        set_u2 = 0;
    }
    if(U1U2_CONTROL == mode)
    {
        try
        {
            tt15_sp->writeSpeed(0, 0, set_u1, set_u2, 1);
        }
        catch(std::exception &ec)
        {
            std::cout << ec.what();
        }
    }
    else
    {
        try
        {
            tt15_sp->writeSpeed(set_v, set_omega, 0, 0, 2);
        }
        catch(std::exception &ec)
        {
            std::cout << ec.what();
        }
    }
}

void TT15Driver::joyCallBack(const sensor_msgs::Joy::ConstPtr& joy)
{
    int enable_pressed = joy->buttons[enable_button_];
    if(1 == enable_pressed)
    {
        enable_flag_ = true;
        ROS_INFO("enable joy");
    }
    else
    {
        enable_flag_ = false;
        tt15_sp->writeSpeed(0, 0, 0, 0, 1);
    }
    int X = joy->buttons[buttonX_];
    int Y = joy->buttons[buttonY_];
    int fork_signal = joy->axes[axesFork_];
    if(1 == X)
    {
        mode = CMDVEL_CONTROL;
        ROS_INFO("current mode: CMDVEL_CONTROL");
    }
    else if(1 == Y)
    {
        mode = U1U2_CONTROL;
        ROS_INFO("current mode: U1U2_CONTROL");
    }
    else
    {
        return;
    }
}

bool TT15Driver::updateOdom()
{
    bool ret = tt15_sp->readOdom(odom_x, odom_y, odom_theta, odom_v, odom_omega);
    ROS_INFO("odom_v %f, odom_omega %f", odom_v, odom_omega);
    return ret;
}

void TT15Driver::publishMsg()
{
    ros::Time current_time = ros::Time::now();
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, odom_theta);

    odom_msg.header.seq = seq_counter_;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "encoder_odom";
    odom_msg.child_frame_id = "base_footprint";
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation.x = q.x;
    odom_msg.pose.pose.orientation.y = q.y;
    odom_msg.pose.pose.orientation.z = q.z;
    odom_msg.pose.pose.orientation.w = q.w;
    odom_msg.pose.covariance = {0};
    odom_msg.twist.twist.linear.x = odom_v;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.linear.z = 0;
    odom_msg.twist.twist.angular.x = 0;
    odom_msg.twist.twist.angular.y = 0;
    odom_msg.twist.twist.angular.z = odom_omega;
    odom_msg.twist.covariance = {0};

    odom_pub_.publish(odom_msg);
    // ROS_ERROR("encoder_odom: x: %.4lf m, y: %.4lf m, theta: %.4lf rad", odom_x, odom_y, odom_theta);
    // ROS_ERROR("seq: %d\n\tX: %.4f, Y: %.4f, THETA: %.4f, V: %.4f, OMEGA: %.4f", seq_counter_, odom_x, odom_y, odom_theta, odom_v, odom_omega);
    // publish tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "encoder_odom";
    odom_trans.child_frame_id = "base_footprint";

    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromYaw(odom_theta);
    odom_trans.transform.translation.x = odom_x;
    odom_trans.transform.translation.y = odom_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // odom_broadcaster_.sendTransform(odom_trans);

}

int main(int agrc,char **argv)
{
    ros::init(agrc,argv,"tt15_driver");
    TT15Driver tt15_driver;
    ros::Rate loop_rate(50);
    // serialInit();
    
    while(ros::ok())
    {   
        if(tt15_driver.updateOdom())
        {
            // ROS_INFO("Update odom data successfully");    
            // std::ofstream v_profile_out;
            // tt15_driver.time_current = std::chrono::steady_clock::now();
            // std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(tt15_driver.time_current-tt15_driver.time_start);
            // v_profile_out.open("v_profile000.csv", std::ios::app);
            // v_profile_out << std::setiosflags(std::ios::fixed) << std::setprecision(5);
            // v_profile_out  << time_used.count() << "," << tt15_driver.set_v << "," << tt15_driver.set_omega << "," << tt15_driver.odom_v << "," << tt15_driver.odom_omega << "\n";
            // v_profile_out.close();
        }
        else
        {
            // ROS_INFO("Fail to update odom data");
        }
        tt15_driver.publishMsg();
        tt15_driver.updateSeq();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}




