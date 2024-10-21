//
// Created by jvaccaro on 5/10/19.
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

#include "lib_kongsberg_em/kongsberg_em2040.h"
#include <tf/transform_broadcaster.h>
#include <mutex>

namespace kongsberg_em{
struct KongsbergEM2040Private{
  ros::NodeHandle nh_;

  // Callback given to the ROS node to send data using the KController
  // UDP connection
  SendKCtrlDataFunc send_kctrl_data_;

  ros::ServiceServer ping_srv_;
  ros::ServiceServer power_srv_;
  ros::ServiceServer settings_srv_;
  ros::ServiceServer multi_settings_srv_;
  ros::ServiceServer read_settings_srv_;
  ros::ServiceServer bist_srv_;
  ros::ServiceServer xml_srv_;

  ros::Publisher mbraw_pub_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher offset_pub_;
  ros::Publisher mrz_pub_;
  ros::Publisher mrz_full_pub_;

  ros::Publisher kssis_pub_;
  ros::Publisher kmstatus_pub_;
  ros::Publisher kmall_record_pub_;
  ros::Publisher rt_params_pub_;

  // The MRZ data from the KMALL format are provided wrt
  // "Vessel Coordinate System" as configured in installation parameters
  std::string mrz_frame_id_;

  // KCtrl startup info
  ds_kongsberg_msgs::KongsbergStatus m_status;
  ds_kongsberg_msgs::KongsbergRuntimeParams m_rtParams;

  ros::Timer kctrl_timer;
  ros::Timer kmall_timer;
  ros::Timer pu_powered_timer;
  ros::Timer pu_connected_timer;
  ros::Timer pinging_timer;
  std::ofstream kmall_stream;
  uint16_t pck_cnt = 0;

  // Diagnostics
  bool m_decodingFailed = false;

  // If there's a larger kmall datagram that gets partitioned, we need to fix it
  ds_core_msgs::RawData kmall_partitioned;
  uint16_t kmall_dgmNum = 0;
  uint16_t kmall_numOfDgms = 0;

  // Data logging
  bool save_kmall_files;

  // All size values in bytes
  long int kmall_buffer_size, kmall_max_buffer_size, kmall_file_size, kmall_max_file_size;

  //BIST
  std::vector<std::string> bist_tests;
  std::stringstream bist_summary_stream;

};
}
