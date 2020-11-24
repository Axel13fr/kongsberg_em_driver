#include "lib_kongsberg_em/kongsberg_em2040.h"
#include "nodeudpsocket.h"
#include <librosasio/asio_callbackqueue.h>

int main(int argc, char* argv[])
{
  // Boost Event loop handler
#if BOOST_VERSION <= 106501
  // ROS Melodic
  boost::asio::io_service io_context;
#else
  // ROS Noetic
  boost::asio::io_context io_context;
#endif
  // Static Function Call doing the magic to unify event loops
  AsioCallbackQueue::replaceGlobalQueue(io_context);

  // ROS setup
  ros::init(argc, argv, "test_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("kongsberg_em2040");

  // UDP sockets for sonar communication
  NodeUdpSocket kctrl_socket(io_context, "kcontroller", nh);
  NodeUdpSocket kmall_socket(io_context, "kmall", nh);

  // The driver will send commands using the kctrl socket
  auto sendKCtrlData = boost::bind(&NodeUdpSocket::sendData, &kctrl_socket, _1);
  kongsberg_em::KongsbergEM2040 driver(nh, sendKCtrlData);
  driver.setupAll();
  // The driver will receive data from the sockets using these call backs
  kctrl_socket.setReceiveCb(boost::bind(&kongsberg_em::KongsbergEM2040::_on_kctrl_data, &driver,_1));
  kmall_socket.setReceiveCb(boost::bind(&kongsberg_em::KongsbergEM2040::_on_kmall_data, &driver,_1));

  io_context.run();
  ros::shutdown();

  return 0;
}
