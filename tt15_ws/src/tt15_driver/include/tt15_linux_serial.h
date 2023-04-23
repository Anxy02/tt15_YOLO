#ifndef TT15_LINUX_SERIAL_H
#define TT15_LINUX_SERIAL_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <string>
#include <vector>

class TT15SerialPort
{
public:
	//Constructor
    TT15SerialPort(const std::string &port_name);
	//Destructor
	~TT15SerialPort();
	
	bool writeSpeed(double set_v, double set_omega, double set_u1, double set_u2, unsigned char work_mode);
	bool readOdom(double &x,double &y,double &theta, double &v, double &omega);
	unsigned char getCrc8(unsigned char *ptr, unsigned short len);
	double convertTheta(double theta);
	
private:
	//Initialize port
	bool serialInit(const std::string port, const unsigned int char_size);

	//io_service Object
	boost::asio::io_service m_ios;

	//Serial port Object
	boost::asio::serial_port *pSerialPort;

	//For save com name
	std::string m_port;

	//Serial_port function exception
	boost::system::error_code ec;
	const unsigned char header[2] = {0x0d, 0x0a};
	const unsigned char ender[2] = {0x55, 0xaa};

	// 发送轮速和货叉控制信号union
	union SendDataFrame
	{
		unsigned char data_char_list[24];
		struct DataStruct
		{
			unsigned char header_[2];
			short length;
			float set_u1;// 控制左轮转速
			float set_u2;// 控制右轮转速
			float set_v;// 差分线速度double
			float set_omega;// 差分角速度
			unsigned char work_mode;// 发送轮速还是差分控制的模式切换，1表示轮速，2表示v和w
			unsigned char crc_data;
			unsigned char ender_[2];
		}__attribute__((packed)) data_struct;		
	} send_data_frame;

	// 接收机器人位姿和速度以及货物到位信号
	union ReceiveDataFrame
	{
		unsigned char rec_char_list[27];
		struct DataStruct
		{
			unsigned char header_[2];
			short length;
			float odomX;
			float odomY;
			float odomTheta;// 机器人位姿
			float odomV;// 线速度
			float odomOmega;// 角速度
			unsigned char crc_data;
			unsigned char ender_[2];
		}__attribute__((packed)) rec_struct;
	} rec_data_frame;

};

#endif
