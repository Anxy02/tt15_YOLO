#include "tt15_linux_serial.h"
#define PI 3.141592653589

using namespace std;
using namespace boost::asio;

//Define Constructor function
TT15SerialPort::TT15SerialPort(const string &port_name)
	:pSerialPort(NULL)
{
    pSerialPort = new serial_port(m_ios);
	if (pSerialPort)
	{
		ROS_INFO("Initializing serial port");
		while(!serialInit(port_name, 8));
		ROS_INFO("Initialize serial port successfully");
	}
} 

//Define destructor function
TT15SerialPort::~TT15SerialPort()
{
    if(pSerialPort)
    {
        delete pSerialPort;
    }     
}


//Initialize port
bool TT15SerialPort::serialInit(const string port, const unsigned int char_size)
{
	//New not success
	if (!pSerialPort){
		return false;
	}

    //Open Serial port object
    pSerialPort->open(port, ec);
	if(ec)	return false;

	//Set port argument
	pSerialPort->set_option(serial_port::baud_rate(115200), ec);
	pSerialPort->set_option(serial_port::flow_control(serial_port::flow_control::none), ec);
	pSerialPort->set_option(serial_port::parity(serial_port::parity::none), ec);
	pSerialPort->set_option(serial_port::stop_bits(serial_port::stop_bits::one), ec);
	pSerialPort->set_option(serial_port::character_size(char_size), ec);
    if(ec)	return false;

	return true;
}

/********************************************************
函数功能：将对机器人的线速度和角速度打包发送给下位机
入口参数：机器人线速度、角速度、货叉控制信号和叉车控制模式
出口参数：
********************************************************/
bool TT15SerialPort::writeSpeed(double set_v, double set_omega, double set_u1, double set_u2, unsigned char work_mode)
{
    int i = 0;
    // 设置消息头
    for(i = 0; i < 2; i++)
        send_data_frame.data_struct.header_[i] = header[i];             //buf[0]  buf[1]
    // 长度
    send_data_frame.data_struct.length = 21;

    // 根据工作模式，设置机器人线速度和角速度或者两轮转速
    switch(work_mode){
        case 1:
            send_data_frame.data_struct.set_u1 = set_u1;
            send_data_frame.data_struct.set_u2 = set_u2;
            break;
        case 2:    
            send_data_frame.data_struct.set_v = set_v;
            send_data_frame.data_struct.set_omega = set_omega;
            ROS_INFO("set_v:%f",set_v);
            break;
    }

    // 叉车控制模式，0为平板远程控制，1为主从控制模式
    send_data_frame.data_struct.work_mode = work_mode;       //buf[8]

    // 设置校验值、消息尾
    send_data_frame.data_struct.crc_data = getCrc8(send_data_frame.data_char_list, send_data_frame.data_struct.length);//buf[9]
    send_data_frame.data_struct.ender_[0] = ender[0];     //buf[10]
    send_data_frame.data_struct.ender_[1] = ender[1];     //buf[10]

    // 通过串口下发数据
    boost::asio::write(*pSerialPort, boost::asio::buffer(send_data_frame.data_char_list), ec);
    ROS_INFO("I have sent message");
    if(ec)
    {
        ROS_ERROR("write message error!");
        return false;
    }
    return true;
}

/********************************************************
函数功能：从下位机读取里程计数据以及货物到位开关信号
********************************************************/
bool TT15SerialPort::readOdom(double &x,double &y,double &theta, double &v, double &omega)
{
    
    char i = 0;
    unsigned char checkSum;
    //=========================================================
    //此段代码可以读数据的结尾，进而来进行读取数据的头部
    try
    {
        boost::asio::streambuf response;
        boost::asio::read_until(*pSerialPort, response, "\r\n",ec);   
        copy(istream_iterator<unsigned char>(istream(&response)>>noskipws),
        istream_iterator<unsigned char>(),
        rec_data_frame.rec_char_list); 
    }  
    catch(boost::system::system_error &ec)
    {
        ROS_INFO("read_until error");
    } 
    //=========================================================        
    // ROS_INFO("recieve header[0]: %x", rec_data_frame.rec_struct.header_[0]);
    // ROS_INFO("recieve header[1]: %x", rec_data_frame.rec_struct.header_[1]);
    // 检查信息头
    if (rec_data_frame.rec_struct.header_[0]!= header[0] || rec_data_frame.rec_struct.header_[1] != header[1])   //buf[0] buf[1]
    {
        ROS_ERROR("Received message header error!");
        return false;
    }
    // 数据长度
    short length = rec_data_frame.rec_struct.length;                                 //buf[2]
    // ROS_INFO("length: %d", length);
    // 检查信息校验值
    checkSum = getCrc8(rec_data_frame.rec_char_list, length);       // length 需要减掉最后三位
    // ROS_INFO("checkSum:%d",checkSum);
    // ROS_INFO("receive crc:%d", rec_data_frame.rec_struct.crc_data);
    if (checkSum != rec_data_frame.rec_struct.crc_data)                 //buf[] 串口接收
    {
        ROS_ERROR("Received data check sum error!");
        return false;
    }     

    // 读取机器人位姿数据
    x = rec_data_frame.rec_struct.odomX;
    y = rec_data_frame.rec_struct.odomY;
    theta = rec_data_frame.rec_struct.odomTheta;
    theta = convertTheta(theta);
    // ROS_INFO("rec_x: %f rec_y: %f rec_theta: %f", x, y, theta);

    // 读取速度数据
    v = rec_data_frame.rec_struct.odomV;
    omega = rec_data_frame.rec_struct.odomOmega;
    // ROS_INFO("rec_v: %f", v);
    // ROS_INFO("rec_omega: %f", omega);
    return true;
}

/********************************************************
函数功能：获得8位循环冗余校验值
入口参数：数组地址、长度
出口参数：校验值
********************************************************/
unsigned char TT15SerialPort::getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else 
                crc >>= 1;
        }
    }
    return crc;
}


double TT15SerialPort::convertTheta(double theta)
{
    while(theta > PI)
        theta -= 2*PI;
    while(theta < -PI)
        theta += 2*PI;
    return theta;
}
