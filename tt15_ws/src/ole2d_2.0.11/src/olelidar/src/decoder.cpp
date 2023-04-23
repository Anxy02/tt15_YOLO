#include "constants.h"

#include <dynamic_reconfigure/server.h>

#include <sensor_msgs/LaserScan.h>

#include <olei_msgs/oleiPacket.h>
#include <olei_msgs/oleiScan.h>
#include <olelidar/oleiPuckConfig.h>

#include "driver.cpp"
// here maskoff waring of macro 'ROS_LOG'
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"

namespace olelidar
{

  using namespace sensor_msgs;
  using namespace olei_msgs;

  class Decoder
  {
  public:
    explicit Decoder(const ros::NodeHandle &pnh);

    Decoder(const Decoder &) = delete;
    Decoder operator=(const Decoder &) = delete;

    void PacketCb(const oleiPacketConstPtr &packet_msg);
    void ConfigCb(oleiPuckConfig &config, int level);
    void PublishMsg(ros::Publisher *pub, std::vector<uint16_t> &packet_r, std::vector<uint16_t> &packet_i, ros::Time t);

  private:
    /// 9.3.1.3 Data Point
    /// A data point is a measurement by one laser channel of a relection of a
    /// laser pulse
    struct DataPoint
    {
      uint16_t azimuth;      //unit: 0.01 degree
      uint16_t distance;     //unit: 1mm
      uint16_t reflectivity; //unit: 1 percent,100 is the reflet of white target
      uint16_t distance2;    //rsv for dual return
    } __attribute__((packed));
    static_assert(sizeof(DataPoint) == 8, "sizeof(DataPoint) != 8");

    struct FiringSequence
    {
      DataPoint points[kFiringsPerSequence]; // 8
    } __attribute__((packed));
    static_assert(sizeof(FiringSequence) == 8, "sizeof(FiringSequence) != 8");

    struct DataHead
    {
      uint8_t magic[4];
      uint8_t version[2];
      uint8_t scale;
      uint8_t oem[3];
      uint8_t model[12];
      uint8_t code[2];
      uint8_t hw[2];
      uint8_t sw[2];
      uint32_t timestamp;
      uint16_t rpm;
      uint8_t flag[2];
      uint32_t rsv;
    } __attribute__((packed));
    static_assert(sizeof(DataHead) == 40, "sizeof(DataBlock) != 40");

    struct DataBlock
    {
      FiringSequence sequences[kSequencesPerBlock]; // 8 * 1
    } __attribute__((packed));
    static_assert(sizeof(DataBlock) == 8, "sizeof(DataBlock) != 8");

    /// 9.3.1.5 Packet format for 2d
    struct Packet
    {
      DataHead head;
      DataBlock blocks[kBlocksPerPacket]; // 8 * 150

    } __attribute__((packed));
    static_assert(sizeof(Packet) == 1240, "sizeof(DataBlock) != 1240");
    static_assert(sizeof(Packet) == sizeof(olei_msgs::oleiPacket().data), "sizeof(Packet) != 1240");

    void DecodeAndFill(const Packet *const packet_buf, uint64_t);

  private:
    bool CheckFactoryBytes(const Packet *const packet);

    // ROS related parameters
    uint32_t laststamp;
    uint32_t scantime;
    ros::Time start;
    bool is_time_base_{false};
    ros::Time local_timestamp_base_;
    uint32_t inner_timestamp_base_;

    std::string frame_id_;
    int range_min_=200;
    int range_max_;
    int ange_start_;
    int ange_end_;
    bool inverted_;
    int poly_=1;
    ros::NodeHandle pnh_;
    // sub driver topic(msg)
    ros::Subscriber packet_sub_;
    // pub laserscan message
    ros::Publisher scan_pub_;

    // dynamic param server
    dynamic_reconfigure::Server<oleiPuckConfig> cfg_server_;
    oleiPuckConfig config_;

    // add vector for laserscan
    std::vector<uint16_t> scanAngleVec_;
    std::vector<uint16_t> scanRangeVec_;
    std::vector<uint16_t> scanIntensityVec_;

    std::vector<uint16_t> scanAngleInVec_;
    std::vector<uint16_t> scanRangeInVec_;
    std::vector<uint16_t> scanIntensityInVec_;

    std::vector<float> scanRangeBuffer;
    std::vector<float> scanintensitiesBuffer;
    uint16_t azimuthLast_;
    uint16_t azimuthNow_;
    uint16_t azimuthFirst_;

    // laserscan msg
    uint32_t scanMsgSeq_;
    //电机频率
    float frequency;
    //雷达型号类型
    unsigned char lidarType=0x01; 
    //电机方向定义
    int direction;

    ros::Time log_time_ = ros::Time::now();

    std::shared_ptr<Driver> drv_;
  };

  Decoder::Decoder(const ros::NodeHandle &pnh)
      : pnh_(pnh), cfg_server_(pnh)
  {
    // get param from cfg file at node start
    pnh_.param<std::string>("frame_id", frame_id_, "olelidar");
    pnh_.param<int>("r_max", range_max_, 30);
    ROS_INFO("===========================");
    ROS_INFO("Frame_id: %s", frame_id_.c_str());
    ROS_INFO("Topic: /%s/scan", frame_id_.c_str());
    ROS_INFO("Range: [%d ~ %d] mm",range_min_, range_max_*1000);
    ROS_INFO("===========================");
    start = ros::Time::now();
    // dynamic callback
    cfg_server_.setCallback(boost::bind(&Decoder::ConfigCb, this, _1, _2));
    // packet receive
    azimuthLast_ = 0;
    azimuthNow_ = 0;
    azimuthFirst_ = 0xFFFF;
    // laserscan msg init
    scanMsgSeq_ = 0;
    direction = 0;

    drv_ = std::make_shared<Driver>(pnh);
    
    scan_pub_ = pnh_.advertise<LaserScan>("scan", 100);
#ifdef DRIVER_MODULE
    packet_sub_ = pnh_.subscribe<oleiPacket>("packet", 100, &Decoder::PacketCb, this, ros::TransportHints().tcpNoDelay(true));
#endif
    drv_->setCallback(std::bind(&Decoder::PacketCb, this, std::placeholders::_1));

    ROS_INFO("Drive Ver:2.0.11");
    ROS_INFO("Decoder initialized");

  }

  void Decoder::PublishMsg(ros::Publisher *pub, std::vector<uint16_t> &packet_r, std::vector<uint16_t> &packet_i, ros::Time lidar_time)
  {
    sensor_msgs::LaserScan scanMsg;
    /*
  std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
  float32 angle_min
  float32 angle_max
  float32 angle_increment
  float32 time_increment
  float32 scan_time
  float32 range_min
  float32 range_max
  float32[] ranges
  float32[] intensities
  */

    int min = config_.angle_min * 100 + 18000;
    int max = config_.angle_max * 100 + 18000;

    scanRangeBuffer.clear();
    scanintensitiesBuffer.clear();

    for (uint16_t i = 0; i < scanAngleInVec_.size(); i++)
    {
      // 过滤出指定角度范围内点云
      int angle = scanAngleInVec_[i];
      if (angle >= min && angle <= max && i % poly_ == 0)
      {
        float range = scanRangeInVec_[i] * 0.001f;
        float intensities = scanIntensityInVec_[i] * 1.0f;

        scanRangeBuffer.push_back(range);
        scanintensitiesBuffer.push_back(intensities);
      }
    }

    float bufferlen = scanRangeBuffer.size();
    scanMsg.ranges.resize(bufferlen);
    scanMsg.intensities.resize(bufferlen);

	if(direction ==0)		//电机旋转顺时针时，转换右手坐标系法则
	{
		reverse(scanRangeBuffer.begin(),scanRangeBuffer.end());
		reverse(scanintensitiesBuffer.begin(),scanintensitiesBuffer.end());
	}
	
    for (uint16_t i = 0; i < bufferlen; i++)
    {
        scanMsg.ranges[i] = scanRangeBuffer[i];
        scanMsg.intensities[i] = scanintensitiesBuffer[i];
    }

    float len = scanMsg.ranges.size();
    //扫描顺序自增ID序列
    scanMsg.header.seq = scanMsgSeq_;
    scanMsgSeq_++;
    //激光数据时间戳
    //scanMsg.header.stamp = lidar_time;
    scanMsg.header.stamp = ros::Time::now();
    //FrameId
    scanMsg.header.frame_id = frame_id_.c_str();
    //定义开始角度和结束角度
    scanMsg.angle_min = deg2rad(config_.angle_min);
    scanMsg.angle_max = deg2rad(config_.angle_max);
    //定义测距的最小值和最大值单位:m
    scanMsg.range_min = config_.range_min;
    scanMsg.range_max = config_.range_max;
    
    float step=(config_.angle_max - config_.angle_min)/len;
    //ROS_INFO("len:%f  step:%f",len,step);
    //角度分辨率
    scanMsg.angle_increment = deg2rad(step);
    //扫描的时间间隔
    scanMsg.scan_time = 1/frequency;
    //时间分辨率（相邻两个角度之间耗费时间）
    scanMsg.time_increment = scanMsg.scan_time/float(len);
 	//修正为雷达内部时钟间隔
 	//scanMsg.scan_time = scantime*0.001f;		//雷达时间戳为毫秒计数，故而转为秒单位应乘以0.001
	//scanMsg.time_increment = scanMsg.scan_time/float(len);
	
    //ROS_INFO("scan_time:%f   time_increment:%f",scanMsg.scan_time,scanMsg.time_increment);
	
	
    uint16_t size=scanAngleInVec_.size();
    uint16_t fb=360/(config_.step);
    if (fb==size){
      pub->publish(scanMsg); //校验当符合点数完整的一帧数据才向外发布话题
    }
    else{
      if(scanMsgSeq_==1) return;
      ROS_INFO("pointCloud frame:%d  size:%d  scanMsgSeq_:%d",fb,size,scanMsgSeq_);
    }
  }

  void Decoder::DecodeAndFill(const Packet *const packet_buf, uint64_t time)
  {

    // For each data block, 150 total
    uint16_t azimuth;
    uint16_t range;
    uint16_t intensity;
    for (int iblk = 0; iblk < kBlocksPerPacket; ++iblk)
    {
      const auto &block = packet_buf->blocks[iblk];

      // simple loop
      azimuth = block.sequences[0].points[0].azimuth;
      range = block.sequences[0].points[0].distance;
      intensity = block.sequences[0].points[0].reflectivity;
      //排除异常点云数据
      if (range > range_max_*1000 || range < range_min_)
      {
        range = 0;
        intensity = 0;
      }
      // azimuth ge 36000 is not valid
      if (azimuth < 0xFF00)
      {
        scanAngleVec_.push_back(azimuth);
        scanRangeVec_.push_back(range);
        scanIntensityVec_.push_back(intensity);
      }
    }
  }

  void Decoder::PacketCb(const oleiPacketConstPtr &packet_msg)
  {
    ros::Time now = ros::Time::now();
    ros::Duration delta_t = now - log_time_;
    log_time_ = now;
    
    const auto *packet_buf = reinterpret_cast<const Packet *>(&(packet_msg->data[0]));
    azimuthNow_ = packet_buf->blocks[0].sequences[0].points[0].azimuth;
    //取得第一个数据包
    if (azimuthFirst_ == 0xFFFF)
    {
      //雷达型号类型
      lidarType = packet_buf->head.code[1];
      azimuthFirst_ = azimuthNow_;
      //取得转速
      int rpm = (packet_buf->head.rpm) & 0x7FFF;
      //取得电机旋转方向
      direction = (packet_buf->head.rpm) >> 15;
      ROS_INFO("rpm:%d  direction:%d  lidarType:%d", rpm, direction,lidarType);
      //是否启用倒装设定
      if (inverted_) direction = !direction;
    }

    if (azimuthLast_ < azimuthNow_)
    {
      DecodeAndFill(packet_buf, packet_msg->stamp.toNSec());
      azimuthLast_ = azimuthNow_;
      return;
    }
    else
    {
      azimuthLast_ = azimuthNow_;
    }
    // scan first half route
    if (azimuthFirst_ >= 200)
    {
      azimuthFirst_ = azimuthNow_;
      return;
    }
    //雷达时间戳
    uint32_t nowstamp = packet_buf->head.timestamp;
    ros::Time lidar_time = packet_msg->stamp;
    
#ifdef TIMESTAMP_DEBUG
    if(!is_time_base_) {
      local_timestamp_base_ = ros::Time::now();
      inner_timestamp_base_ = nowstamp;
      lidar_time = local_timestamp_base_;
      is_time_base_ = true;
    } else {
      uint32_t delta_time = nowstamp - inner_timestamp_base_;
      ros::Duration dur_time;
      ros::Duration delta_t = dur_time.fromNSec(delta_time*1000000);
      lidar_time = local_timestamp_base_ + delta_t;
      //ROS_INFO_STREAM("ros timestamp:" << delta_time << "," << delta_t << "," << lidar_time);
    }
#endif
    scantime = nowstamp - laststamp;
    laststamp = nowstamp;
    //ROS_INFO("inner timestamp: %u, %d", nowstamp, scantime);

    //雷达工作频率
    if(frequency<0.001){
       lidarType = packet_buf->head.code[1]; //雷达型号类型
		if(lidarType==0x01){
		  int rpm = (packet_buf->head.rpm) & 0x7FFF;
		  frequency = rpm / 60.0;
		  config_.step=0.225;    //当雷达型号为0x01类型时，角分辨率为固定值
		}
		else{    
		      if(scanAngleVec_.size()>2) {
					//角度分辨率
					config_.step = (scanAngleVec_[1]-scanAngleVec_[0])/100.0;
					frequency = config_.step * 10000.0/60.0; 
				}
				else{
					return;
				}
		}
		ROS_INFO("frequency: %.0f hz  config_.step:%f",frequency,config_.step);
    }

	//当启用NTP服务时，时间戳重定向为NTP服务时间
	if(packet_buf->head.rsv>0){
		ros::Time ntp(packet_buf->head.rsv, packet_buf->head.timestamp);
		lidar_time=ntp;
	}
    scanAngleInVec_.clear();
    scanRangeInVec_.clear();
    scanIntensityInVec_.clear();

    scanAngleInVec_.assign(scanAngleVec_.begin(), scanAngleVec_.end());
    scanRangeInVec_.assign(scanRangeVec_.begin(), scanRangeVec_.end());
    scanIntensityInVec_.assign(scanIntensityVec_.begin(), scanIntensityVec_.end());

    scanAngleVec_.clear();
    scanRangeVec_.clear();
    scanIntensityVec_.clear();

    //抛出点云数据
    PublishMsg(&scan_pub_, scanRangeInVec_, scanIntensityInVec_, lidar_time);

    //解码原始数据包
    DecodeAndFill(packet_buf, packet_msg->stamp.toNSec());

    //ROS_INFO("Time: %f", (ros::Time::now() - start).toSec());
  }

  void Decoder::ConfigCb(oleiPuckConfig &config, int level)
  {
    // config.min_range = std::min(config.min_range, config.max_range);
    //config.route =4000;
    pnh_.param<int>("ang_start", ange_start_, 0);
    pnh_.param<int>("ang_end", ange_end_, 360);
    pnh_.param<int>("r_max", range_max_, 50);
    pnh_.param<bool>("inverted", inverted_, false);
    config.angle_min = ange_start_;
    config.angle_max = ange_end_;
    config.range_min = range_min_*0.001;
    config.range_max = range_max_;
    config_ = config;


    if (level < 0){
      // topic = scan, msg = LaserScan, queuesize = 1000
      scan_pub_ = pnh_.advertise<LaserScan>("scan", 100);
#ifdef DRIVER_MODULE
      packet_sub_ = pnh_.subscribe<oleiPacket>("packet", 100, &Decoder::PacketCb, this);
#endif
    }
  }
} // namespace olelidar

int main(int argc, char **argv)
{
  ros::init(argc, argv, "olelidar_decoder");
  ros::NodeHandle pnh("~");
  olelidar::Decoder node(pnh);
  ros::spin();
}
