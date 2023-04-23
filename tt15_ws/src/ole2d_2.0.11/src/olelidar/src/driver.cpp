#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cmath>
#include <functional>
#include <thread>

#include "constants.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <olei_msgs/oleiPacket.h>
#include <olei_msgs/oleiScan.h>

// here maskoff waring of macro 'ROS_LOG'
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"

namespace olelidar
{

  using namespace olei_msgs;
  using namespace diagnostic_updater;

  /// Constants
  //static constexpr uint16_t kUdpPort = 2368;
  static constexpr size_t kPacketSize = sizeof(oleiPacket().data);
  static constexpr int kError = -1;

  class Driver
  {
    using data_cb_t =  std::function<void(const oleiPacketConstPtr&)>;
  public:
    explicit Driver(const ros::NodeHandle &pnh);
    ~Driver();

    bool Poll();

    void setCallback(const data_cb_t &cb) { data_cb_ = cb; }

  private:
    bool OpenUdpPort();
    int ReadPacket(oleiPacket &packet);

    // Ethernet relate variables
    std::string device_ip_str_;
    std::string local_ip_str_;
    std::string multiaddr_ip_str_;
    int device_port_;
    in_addr device_ip_;
    int socket_id_{-1};
    ros::Time last_time_;

    // ROS related variables
    ros::NodeHandle pnh_;
    ros::Publisher pub_packet_;

    // Diagnostics updater
    diagnostic_updater::Updater updater_;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> topic_diag_;
    std::vector<oleiPacket> buffer_;
    // double freq_;
    
    // raw data callback
    data_cb_t data_cb_{nullptr};
    std::thread data_thr_;
    bool is_loop_{false};
  };

  Driver::Driver(const ros::NodeHandle &pnh) 
    : pnh_(pnh)
    , last_time_(ros::Time::now())
  {
    ROS_INFO("packet size: %zu", kPacketSize);
    pnh_.param("device_ip", device_ip_str_, std::string("192.168.1.100"));
    pnh_.param("device_port", device_port_, 2368);
    pnh_.param("local_ip", local_ip_str_, std::string("192.168.1.10"));
    pnh_.param("multiaddr", multiaddr_ip_str_, std::string(""));
    ROS_INFO("device_ip: %s:%d", device_ip_str_.c_str(), device_port_);

    if (inet_aton(device_ip_str_.c_str(), &device_ip_) == 0)
    {
      // inet_aton() returns nonzero if the address is valid, zero if not.
      ROS_FATAL("Invalid device ip: %s:%d", device_ip_str_.c_str(), device_port_);
      ros::shutdown();
    }

    // Output
    pub_packet_ = pnh_.advertise<oleiPacket>("packet", 10);

    if (!OpenUdpPort()) ROS_ERROR("Failed to open UDP Port"); 
     
    data_thr_ = std::move(std::thread( [&] {
      is_loop_ = true;
      while (is_loop_) {
          if(!is_loop_) break; 
          Poll();
      }
    })); 
    ROS_INFO("Successfully init olelidar driver");
  }

  Driver::~Driver()
  {
    is_loop_ = false;
    if(data_thr_.joinable()) {
      data_thr_.join();
    }
    if (close(socket_id_)) {
      ROS_INFO("Close socket %d at %s", socket_id_, device_ip_str_.c_str());
    }
    else {
      ROS_ERROR("Failed to close socket %d at %s", socket_id_,device_ip_str_.c_str());
    }
  }

  bool Driver::OpenUdpPort()
  {
    socket_id_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_id_ == -1) {
      perror("socket");
      ROS_ERROR("Failed to create socket");
      return false;
    }

    sockaddr_in my_addr;                              // my address information
    memset(&my_addr, 0, sizeof(my_addr));             // initialize to zeros
    my_addr.sin_family = AF_INET;                     // host byte order
    my_addr.sin_port = htons(uint16_t(device_port_)); // short, in network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY;            // automatically fill in my IP


    if (bind(socket_id_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
    {
      perror("bind"); // TODO: ROS_ERROR errno
      ROS_ERROR("Failed to bind to socket %d", socket_id_);
      return false;
    }
    if(multiaddr_ip_str_!="")
    {
    //加入组播才能接受到组播信息
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(multiaddr_ip_str_.c_str());
    mreq.imr_interface.s_addr = inet_addr(local_ip_str_.c_str());
    int err=setsockopt(socket_id_,IPPROTO_IP,IP_ADD_MEMBERSHIP,&mreq,sizeof(mreq));
    ROS_INFO("AddMultiaddr:%s  local:%s   =>%d ",multiaddr_ip_str_.c_str(),local_ip_str_.c_str(),err);
    }   
    if (fcntl(socket_id_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    {
      perror("non-block");
      ROS_ERROR("Failed to set socket to non-blocking");
      return false;
    }
    return true;
  }

 

  int Driver::ReadPacket(oleiPacket &packet)
  {
    ros::Time time_before = ros::Time::now();

    struct pollfd fds[1];
    fds[0].fd = socket_id_;
    fds[0].events = POLLIN;
    const int timeout_ms = 500; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (true)
    {
      do
      {
        const int retval = poll(fds, 1, timeout_ms);
        if (retval < 0) {
          if (errno != EINTR) ROS_ERROR("poll() error: %s", strerror(errno));
          return kError;
        }
        else if (retval == 0)
        {
          ROS_WARN("olamlidar poll() timeout");
          return kError;
        }

        if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))
        {
          ROS_ERROR("poll() reports olamlidar error");
          return kError;
        }
      } while ((fds[0].revents & POLLIN) == 0);

      // Receive packets that should now be available from the
      // socket using a blocking read.
      const ssize_t nbytes = recvfrom(socket_id_, &packet.data[0], kPacketSize, 0, (sockaddr *)&sender_address, &sender_address_len);
      
      if (nbytes < 0)
      {
        if (errno != EWOULDBLOCK)
        {
          perror("recvfail");
          ROS_ERROR("Failed to read from socket");
          return kError;
        }
      }
      else if (static_cast<size_t>(nbytes) == kPacketSize)
      {
        //else if ((size_t)nbytes == kPacketSize) {
        // read successful,
        // if packet is not from the lidar scanner we selected by IP,
        // continue otherwise we are done
        //ROS_ERROR("Failed to read from socket");

        //ROS_INFO("sender:%d   device:%d",sender_address.sin_addr.s_addr,device_ip_.s_addr); 
        if (device_ip_str_ != "" && sender_address.sin_addr.s_addr != device_ip_.s_addr)
          continue;
        else
          break; // done
      }

      ROS_DEBUG_STREAM("incomplete olei packet read: " << nbytes << " bytes");
    }

    packet.stamp = time_before;

#ifdef TIMESTAMP_DEBUG
    ros::Duration delta = time_before - last_time_;
    ROS_INFO_STREAM("raw data delta time: " << time_before << "," << delta.toSec()*1000);
    last_time_ = time_before;
#endif

    return 0;
  }

  bool Driver::Poll()
  {
    oleiPacket::Ptr packet(new oleiPacket);

    while (true)
    {
      // keep reading until full packet received
      const int rc = ReadPacket(*packet);
      if (rc == 0)
        break; // got a full packet?
      if (rc < 0)
        return false; // end of file reached?
    }

    // publish message using time of last packet read
#ifdef DRIVER_MODULE
    pub_packet_.publish(packet);
#else
    data_cb_(packet);
#endif

    return true;
  }

} // namespace olelidar

#ifdef DRIVER_MODULE
int main(int argc, char **argv)
{
  ros::init(argc, argv, "olelidar_driver");
  ros::NodeHandle pnh("~");

  olelidar::Driver node(pnh);

  while (ros::ok())
  {
    // poll device until end of file
    if (!node.Poll())
    {
      ROS_WARN("Failed to poll device");
      //break;
    }
    ros::spinOnce();
  }
  return 0;
}
#endif
