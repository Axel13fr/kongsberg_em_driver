//
// Created by jvaccaro on 4/11/19.
//
/**
* Copyright 2019 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <ds_core_msgs/RawData.h>
#include "../../src/em_driver_library/EM_datagrams/EMdgmFormat.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/node_handle.h>
#include <pcl_ros/point_cloud.h>

#include "ds_kongsberg_msgs/PingCmd.h"
#include "ds_kongsberg_msgs/PowerCmd.h"
#include "ds_kongsberg_msgs/SettingsCmd.h"
#include "ds_kongsberg_msgs/BistCmd.h"
#include "ds_kongsberg_msgs/LoadXmlCmd.h"
#include "ds_kongsberg_msgs/KongsbergStatus.h"
#include "ds_kongsberg_msgs/KongsbergKMAllRecord.h"

#include <ds_multibeam_msgs/MultibeamRaw.h>
#include <boost/utility.hpp>

#include <functional>

namespace kongsberg_em{

struct KongsbergEM2040Private;
using SendKCtrlDataFunc = std::function<void(const std::string &)>;

class KongsbergEM2040  : boost::noncopyable
{

 public:
  KongsbergEM2040(ros::NodeHandle &nh, SendKCtrlDataFunc send_kctrl_data);
  ~KongsbergEM2040();

  bool parse_data(const ds_core_msgs::RawData &raw);
  bool parse_message(const ds_core_msgs::RawData &raw);
  bool read_kmall_dgm_from_kctrl(int type, const ds_core_msgs::RawData &raw);
  bool parse_ipu(std::vector<std::string> fields);

  /**
   * @brief check_and_append_mpartition
   * @return Returns a bool and a RawData msg.
   * If the datagram is not partitioned, then it returns the datagram.
   * If the datagram is partitioned, then
   *  True means that the datagram is complete (SHOULD BE LOGGED AND PARSED)
   *  False means that the datagram is incomplete (NO LOGGING)
   *
   * This function expects partitions to arrive sequentially for one given message, which is the
   * case when receiving data from the real hardware.
   */
  std::pair<bool, ds_core_msgs::RawData> check_and_append_mpartition(const ds_core_msgs::RawData & );
  bool read_bist_result(ds_core_msgs::RawData& raw);
  uint8_t read_good_bad_missing(std::string);

  void mbraw_to_kmstatus(ds_multibeam_msgs::MultibeamRaw raw);

  // Calls all the ROS setup functions at once(publishers, services, parameters, timers)
  void setupAll();

  void setupServices();
  void setupPublishers();
  void setupParameters();
  void setupTimers();

  void _on_kmall_data(ds_core_msgs::RawData raw);
  void _on_kctrl_data(ds_core_msgs::RawData raw);

private:

  bool _ping_cmd(ds_kongsberg_msgs::PingCmd::Request &req, ds_kongsberg_msgs::PingCmd::Response &res);
  bool _power_cmd(ds_kongsberg_msgs::PowerCmd::Request &req, ds_kongsberg_msgs::PowerCmd::Response &res);
  bool _settings_cmd(ds_kongsberg_msgs::SettingsCmd::Request &req, ds_kongsberg_msgs::SettingsCmd::Response &res);
  bool _bist_cmd(ds_kongsberg_msgs::BistCmd::Request &req, ds_kongsberg_msgs::BistCmd::Response &res);
  bool _load_xml_cmd(ds_kongsberg_msgs::LoadXmlCmd::Request &req, ds_kongsberg_msgs::LoadXmlCmd::Response &res);

  std::string _send_kctrl_command(int cmd);
  template <class T1>
  std::string _send_kctrl_param(std::string param, T1 param_val);
  template <class T1>
  std::string _send_kctrl_param(std::vector<std::string> params, std::vector<T1> vals);
  template <class EMdgmMRZ_S>
  void _read_and_publish_mrz(const ds_kongsberg_msgs::KongsbergKMAllRecord &r,
                             std::vector<uint8_t> &data);

  void _startup_sequence();
  void _print_bist(std::string name, std::string status, std::string msg);
  void _run_next_bist();
  void _new_kmall_file();
  void _write_kmall_data(const std::vector<uint8_t> &data);
  void _write_kctrl_xml(std::vector<uint8_t>& data);
  std::string _read_kctrl_xml(std::string filename);
  void _on_kctrl_timeout(const ros::TimerEvent&);
  void _on_kmall_timeout(const ros::TimerEvent&);
  void _on_pu_powered_timeout(const ros::TimerEvent&);
  void _on_pu_connected_timeout(const ros::TimerEvent&);
  void _on_pinging_timeout(const ros::TimerEvent&);

  static double _timeToLastPartition(const EMdgmHeader *hdr);

  // All object members are hidden inside a pointer to preserve binary compatiblity
  // See https://wiki.qt.io/D-Pointer for explanation
  std::unique_ptr<KongsbergEM2040Private> d;
};

}

