#include "nodeudpsocket.h"
#include <memory>
#include <ros/console.h>
#include <boost/asio/ip/multicast.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

using namespace boost::asio::ip;

NodeUdpSocket::NodeUdpSocket(boost::asio::io_service& io_service, const std::string& name, ros::NodeHandle &nh):
      name_(name)
{
  recv_buffer_.resize(65536);

  if (not nh.hasParam(name + "/udp_rx_port") or not nh.hasParam(name + "/udp_tx_port") or
      not nh.hasParam(name + "/udp_address"))
  {
    ROS_FATAL_STREAM("Missing udp_rx_port or udp_tx_port or udp_address parameter for" << name);
  }

  int udp_rx_port = nh.param(name + "/udp_rx_port", 4444);
  int udp_tx_port = nh.param(name + "/udp_tx_port", 5555);
  std::string udp_address = nh.param(name + "/udp_address", std::string("127.0.0.1"));

  // Create the socket and endpoint
  auto udp_ip = address::from_string(udp_address);
  if (!udp_ip.is_multicast()){
    // Create the socket
    socket_ = std::make_unique<udp::socket>(io_service, udp::endpoint(udp::v4(), udp_rx_port));;
    remote_endpoint_ = udp::endpoint(udp_ip, udp_tx_port);
  } else {
    ROS_ERROR_STREAM("Multicast address "<<udp_address<<" receive only");
    // Create the socket
    socket_ = std::unique_ptr<udp::socket>(new udp::socket(io_service));
    auto listen_address = address::from_string("0.0.0.0");
    udp::endpoint listen_endpoint(listen_address, udp_rx_port);
    socket_->open(listen_endpoint.protocol());
    socket_->set_option(udp::socket::reuse_address(true));
    socket_->bind(listen_endpoint);

    socket_->set_option(multicast::join_group(udp_ip));

    remote_endpoint_ = udp::endpoint();
    remote_endpoint_.port(udp_tx_port);
  }
  // Check for broadcast addresses
  std::vector<std::string> split_add;
  boost::algorithm::split(split_add, udp_address, boost::is_any_of("."));
  if (split_add.size() > 0)
  {
    if (!split_add.back().compare("255"))
    {
      ROS_INFO_STREAM("Setting socket broadcast option");
      boost::asio::socket_base::broadcast option(true);
      socket_->set_option(option);
    }
  }

  receive();
}



void NodeUdpSocket::sendData(const std::string &data)
{
  socket_->async_send_to(boost::asio::buffer(data), remote_endpoint_,
                         boost::bind(&NodeUdpSocket::handle_send, this, boost::asio::placeholders::error));
}

void NodeUdpSocket::receive(void)
{
  recv_buffer_.assign(recv_buffer_.size(), 0);
  socket_->async_receive(boost::asio::buffer(recv_buffer_), 0,
                         boost::bind(&NodeUdpSocket::handle_receive, this, boost::asio::placeholders::error,
                                     boost::asio::placeholders::bytes_transferred));
}

void NodeUdpSocket::handle_receive(const boost::system::error_code &error, std::size_t bytes_transferred)
{
  if (!error || error == boost::asio::error::message_size)
  {
    // Store timestamp as soon as received
    ds_core_msgs::RawData m;
    m.ds_header.io_time = ros::Time::now();
    m.data = std::vector<unsigned char>(recv_buffer_.begin(), recv_buffer_.begin() + bytes_transferred);
    m.data_direction = ds_core_msgs::RawData::DATA_IN;

    if (!recv_cb_.empty())
    {
      recv_cb_(m);
    }

    receive();
    if (error == boost::asio::error::message_size)
    {
      ROS_ERROR_STREAM("UDP Socket message size error.Bytes received" << bytes_transferred);
    }
  }
}

void NodeUdpSocket::handle_send(const boost::system::error_code &error)
{
  if (error.value() != boost::system::errc::success)
  {
    ROS_ERROR_STREAM("Failed sending udp data (" << name_ << "):" << error.message());
  }
}


