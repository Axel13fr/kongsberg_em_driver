#pragma once

#include <ros/node_handle.h>
#include <boost/asio/ip/udp.hpp>
#include <ds_core_msgs/RawData.h>
#include <boost/utility.hpp>

using ReceiveCallback = boost::function<void(const ds_core_msgs::RawData&)>;

class NodeUdpSocket : boost::noncopyable
{
 public:
  NodeUdpSocket(boost::asio::io_service &io_service, const std::string &name,
                ros::NodeHandle &nh);

  void sendData(const std::string &data);
  void setReceiveCb(const ReceiveCallback recv_cb)
  {
    recv_cb_ = recv_cb;
  }

 private:
  std::string name_;

  void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);
  void receive();
  void handle_send(const boost::system::error_code &error);

  std::unique_ptr<boost::asio::ip::udp::socket> socket_;
  boost::asio::ip::udp::endpoint remote_endpoint_;

  std::vector<char> recv_buffer_;
  ReceiveCallback recv_cb_;
};
