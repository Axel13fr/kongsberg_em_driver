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

#include "lib_kongsberg_em/kongsberg_em2040.h"
#include "kongsberg_em2040_private.h"
#include "kongsberg_em2040_util.h"
#include "../../em_driver_library/EM_datagrams/KMALL_mrz_decoder.h"
#include "ds_kongsberg_msgs/KongsbergKSSIS.h"
#include "ds_kongsberg_msgs/KongsbergMRZ.h"
#include "ds_kongsberg_msgs/KongsbergMRZFull.h"
#include "kongsberg_em2040_strings.h"
#include "ds_core_msgs/ClockOffset.h"
#include <regex>

#include <ros/this_node.h>
#include <ros/param.h>


namespace kongsberg_em{
KongsbergEM2040::KongsbergEM2040(ros::NodeHandle &nh, SendKCtrlDataFunc send_kctrl_data)
    : d(std::unique_ptr<KongsbergEM2040Private>(new KongsbergEM2040Private))
{
  d->nh_ = nh;
  d->send_kctrl_data_ = send_kctrl_data;

}

KongsbergEM2040::~KongsbergEM2040(){
  if (d->kmall_stream.is_open()) {
    d->kmall_stream.close();
  }
};

// XXXXXXXXXXXXX
// KCtrl Parsers
// -------------
bool
KongsbergEM2040::parse_message(const ds_core_msgs::RawData& raw)
{

  ds_kongsberg_msgs::KongsbergKSSIS msg;
  // Split on commas and pull specific indexes
  auto str = std::string{ reinterpret_cast<const char*>(raw.data.data()), raw.data.size() };
  auto buf = std::istringstream{ str };
  std::vector<std::string> fields;

  while( buf.good() ) {
    std::string substr;
    getline( buf, substr, ',' );
    fields.push_back( substr );
  }

  if (fields.size() < 3){
    ROS_ERROR_STREAM("Msg too short : "<<str);
    return false;
  }
  if (fields[0] != "$KSSIS"){
    ROS_ERROR_STREAM("Msg doesn't contain $KSSIS : "<<fields[0]);
    return false;
  }

  msg.type = std::stoi(fields[1]);

  switch (msg.type){
    case K_TO_SIS::KCTRL_VERSION :
      if (fields[2] == "KCTRL_VER="){
        break;
      }
    case K_TO_SIS::STATUS_IPI :
      break;
    case K_TO_SIS::STATUS_IPI_PU : {
      d->pu_powered_timer.stop();
      d->pu_powered_timer.start();
      d->m_status.pu_powered = true;
//      std::stringstream ipi(fields[2]);
//      fields.pop_back();
//      while( ipi.good() )
//      {
//        std::string substr;
//        getline( ipi, substr, '~' );
//        if (substr.length()>0)
//          fields.push_back( substr );
//      }
      if (!d->m_status.pu_connected){
        _startup_sequence();
      }
      break;
    }
    case K_TO_SIS::PU_DISCONNECTED : {
      ROS_ERROR_STREAM("PU POWERDOWN DETECTED");
      d->m_status.pu_powered = false;
      break;
    }
    case K_TO_SIS::STATUS_IPU : {
      parse_ipu(fields);
    }
    default :
      if (msg.type>800 && msg.type<900){
        read_kmall_dgm_from_kctrl(msg.type, raw);
        break;
      }
      if (fields[2] != d->m_status.sounder_name){
        ROS_WARN_STREAM("IPU sounder name doesn't match: automatic update from " << d->m_status.sounder_name << " to "
                                                                                 << fields[2]);
        d->m_status.sounder_name = fields[2];
        return false;
      }
      break;
  }

  msg.header = raw.header;
  msg.ds_header = raw.ds_header;
  msg.payload.resize(fields.size() - 2);
  for (size_t i=0; i<msg.payload.size(); i++){
    msg.payload[i] = fields[i+2];
  }
  d->kssis_pub_.publish(msg);
  return true;
}
bool
KongsbergEM2040::parse_ipu(std::vector<std::string> fields)
{
  if (fields.size() != 15) {
    ROS_ERROR_STREAM("IPU size too small " << fields.size());
    return false;
  }

  if (fields[2] != d->m_status.sounder_name){
    ROS_WARN_STREAM("IPU sounder name doesn't match: automatic update from " << d->m_status.sounder_name << " to "
                                                                             << fields[2]);
    d->m_status.sounder_name = fields[2];
    return false;
  }
//  d_ptr_->m_status.cpu_temperature = read_good_bad_missing(fields[4]);
//  d_ptr_->m_status.position_1 = read_good_bad_missing(fields[5]);
//  d_ptr_->m_status.position_2 = read_good_bad_missing(fields[6]);
//  d_ptr_->m_status.position_3 = read_good_bad_missing(fields[7]);
//  d_ptr_->m_status.attitude_1 = read_good_bad_missing(fields[8]);
//  d_ptr_->m_status.attitude_2 = read_good_bad_missing(fields[9]);
//  d_ptr_->m_status.depth = read_good_bad_missing(fields[10]);
//  d_ptr_->m_status.svp = read_good_bad_missing(fields[11]);
//  d_ptr_->m_status.time_sync = read_good_bad_missing(fields[12]);
//  d_ptr_->m_status.pps = read_good_bad_missing(fields[13]);
  d->m_status.cpu_temperature = fields[4];
  d->m_status.position_1 = fields[5];
  d->m_status.position_2 = fields[6];
  d->m_status.position_3 = fields[7];
  d->m_status.attitude_1 = fields[8];
  d->m_status.attitude_2 = fields[9];
  d->m_status.depth = fields[10];
  d->m_status.svp = fields[11];
  d->m_status.time_sync = fields[12];
  d->m_status.pps = fields[13];

  d->pu_connected_timer.stop();
  d->pu_connected_timer.start();
  d->m_status.pu_connected = true;
  d->kmstatus_pub_.publish(d->m_status);
  return true;
}

uint8_t
KongsbergEM2040::read_good_bad_missing(std::string msg)
{
  (void)msg;
//  boost::smatch results;
//  auto good = boost::regex{ "=OK;G" };
//  auto missing = boost::regex{ "=MISSING;R" };
//  auto activebad = boost::regex{ "=ACTIVE-BAD;R" };
//  auto bad = boost::regex{ "=BAD;R" };
//  auto off = boost::regex{ "=OFF;B" };
//  if (boost::regex_search(msg, results, good)){
//    return ds_kongsberg_msgs::KongsbergStatus::SENSOR_OK;
//  }
//  if (boost::regex_search(msg, results, missing)){
//    return ds_kongsberg_msgs::KongsbergStatus::SENSOR_MISSING;
//  }
//  if (boost::regex_search(msg, results, activebad)
//      || boost::regex_search(msg, results, bad)){
//    return ds_kongsberg_msgs::KongsbergStatus::SENSOR_BAD;
//  }
//  if (boost::regex_search(msg, results, off)){
//    return ds_kongsberg_msgs::KongsbergStatus::SENSOR_OFF;
//  }
  return 0;
}

bool
KongsbergEM2040::read_bist_result(ds_core_msgs::RawData& raw)
{
  EMdgm_f::dgm_IB r{};
  uint8_t* ptr = raw.data.data();

  r = *reinterpret_cast<EMdgm_f::dgm_IB*>(ptr);
  size_t index = sizeof(r) - 2;
  auto max_index = raw.data.size();
  if (max_index < index){
    return false;
  }


  auto message = std::string{ reinterpret_cast<const char*>(ptr + index), max_index - index };
  std::string name;
  bist_strings b;
  name = b.get_name_from_code(r.BISTNumber);
  ROS_ERROR_STREAM("RECEIVED BIST "<<name);
  std::string status;
  if (r.BISTStatus == 0)
    status = "PASS";
  else if (r.BISTStatus < 0)
    status = "ERROR";
  else if (r.BISTStatus > 0)
    status = "WARNING";

  if (d->m_status.bist_running){
    _print_bist(name, status, message);
    _run_next_bist();
  }
  return true;
}
bool
KongsbergEM2040::read_kmall_dgm_from_kctrl(int type,const ds_core_msgs::RawData& raw)
{
  ds_core_msgs::RawData stripped_raw{};
  stripped_raw.header = raw.header;
  stripped_raw.ds_header = raw.ds_header;


  std::string front = "$KSSIS," + std::to_string(type) + "," + d->m_status.sounder_name + ",";
  auto index = front.length();
  auto max_index = raw.data.size();
  if (index > max_index){
    return false;
  }
  stripped_raw.data.resize(max_index-index);

  for (size_t i=0; i<max_index-index; i++){
    stripped_raw.data[i] = raw.data[i+index];
  }
  switch (type){
    case K_TO_SIS::XML :
      _write_kctrl_xml(stripped_raw.data);
      return true;
    case K_TO_SIS::BIST_RESULT :
      read_bist_result(stripped_raw);
      return true;
    default:
      return false;
  }
}

// XXXXXXXXXXXXX
// KMAll parsers
// -------------
bool
KongsbergEM2040::parse_data(const ds_core_msgs::RawData& raw)
{
  auto data_size = raw.data.size();
  auto min_size = sizeof(EMdgm_f::EMdgmHeader);
  if (data_size < min_size){
    ROS_ERROR_STREAM("Raw Data received less than minimum size "<<min_size<<" bytes");
    return false;
  }

  const uint8_t* bytes_ptr = raw.data.data();
  auto hdr = reinterpret_cast<const EMdgm_f::EMdgmHeader*>(bytes_ptr);
  //ROS_ERROR_STREAM("TIME: "<<hdr->time_sec);
  std::string msg_type(hdr->dgmType, hdr->dgmType + sizeof(hdr->dgmType)/sizeof(hdr->dgmType[0]));
  const uint8_t dgmVersion = hdr->dgmVersion;

  ds_kongsberg_msgs::KongsbergKMAllRecord r;
  r.ds_header = raw.ds_header;
  r.header.stamp = ros::Time(hdr->time_sec, hdr->time_nanosec);
  r.record_type = msg_type;
  r.kmall_filename = d->m_status.kmall_filename;
  r.record_size = data_size;

  if (data_size != hdr->numBytesDgm){
    ROS_ERROR_STREAM("hdr->dgmType ("<< r.record_type << ") data.size() ("<<data_size<<") != hdr->numBytes ("<<hdr->numBytesDgm<<")");
    return false;
  }

  if (msg_type==EMdgm_f::EM_DGM_I_INSTALLATION_PARAM) {
    r.record_name = "EM_DGM_I_INSTALLATION_PARAM";
    _new_kmall_file();
    _write_kmall_data(raw.data);
  }
  else if (msg_type==EMdgm_f::EM_DGM_I_OP_RUNTIME) {
    r.record_name = "EM_DGM_I_OP_RUNTIME";
    _write_kmall_data(raw.data);
  }
  else if (msg_type==EMdgm_f::EM_DGM_S_POSITION){
    r.record_name = "EM_DGM_S_POSITION";
    _write_kmall_data(raw.data);
  }
  else if (msg_type==EMdgm_f::EM_DGM_S_KM_BINARY){
    r.record_name = "EM_DGM_S_KM_BINARY";
    _write_kmall_data(raw.data);
  }
  else if (msg_type==EMdgm_f::EM_DGM_S_SOUND_VELOCITY_PROFILE){
    r.record_name = "EM_DGM_S_SOUND_VELOCITY_PROFILE";
    _write_kmall_data(raw.data);
  }
  else if (msg_type==EMdgm_f::EM_DGM_S_SOUND_VELOCITY_TRANSDUCER){
    r.record_name = "EM_DGM_S_SOUND_VELOCITY_TRANSDUCER";
    _write_kmall_data(raw.data);
  }
  else if (msg_type==EMdgm_f::EM_DGM_S_CLOCK){
    r.record_name = "EM_DGM_S_CLOCK";
    ds_core_msgs::ClockOffset offset;
    offset.header = r.header;
    offset.ds_header = r.ds_header;
    offset.device_stamp_minus_ros_stamp_sec = offset.header.stamp.toSec() - offset.ds_header.io_time.toSec();
    d->offset_pub_.publish(offset);
    _write_kmall_data(raw.data);
  }
  else if (msg_type==EMdgm_f::EM_DGM_S_DEPTH){
    r.record_name = "EM_DGM_S_DEPTH";
    _write_kmall_data(raw.data);
  }
  else if (msg_type==EMdgm_f::EM_DGM_S_HEIGHT){

    r.record_name = "EM_DGM_S_HEIGHT";
    _write_kmall_data(raw.data);
  }
  else if (msg_type==EMdgm_f::EM_DGM_M_RANGE_AND_DEPTH){
    d->pinging_timer.stop();
    d->pinging_timer.start();
    d->m_status.pinging = true;
    r.record_name = "EM_DGM_M_RANGE_AND_DEPTH";
    r.header.frame_id = d->mrz_frame_id_;
    bool full_data = false;
    ds_core_msgs::RawData logme{};
    std::tie(full_data, logme) = check_and_append_mpartition(raw);
    if (full_data){
      _write_kmall_data(logme.data);
      // Assume that the EMdgmMpartition_def structure will not change so
      // it's safe to collect and write all partitions of this structure even without a
      // compatible version: inform but not try to deserialize until version check
      if (dgmVersion == EMdgm_f::MRZ_VERSION)
      {
        _read_and_publish_mrz<EMdgm_f::EMdgmMRZ>(r,logme.data);
      }
      else if (dgmVersion == EMdgm_h::MRZ_VERSION)
      {
        _read_and_publish_mrz<EMdgm_h::EMdgmMRZ>(r, logme.data);
      }
      else if (dgmVersion == EMdgm_i::MRZ_VERSION)
      {
        _read_and_publish_mrz<EMdgm_i::EMdgmMRZ>(r,logme.data);
      }else{
        ROS_ERROR_STREAM("Received MRZ unsupported message version "
                         << std::to_string(dgmVersion));
        return false;
      }
    } else {
      r.record_name = "EM_DGM_M_RANGE_AND_DEPTH P";
    }
  }
  else if (msg_type==EMdgm_f::EM_DGM_M_WATER_COLUMN){
    r.record_name = "EM_DGM_M_WATER_COLUMN";
    bool full_data = false;
    ds_core_msgs::RawData logme{};
    std::tie(full_data, logme) = check_and_append_mpartition(raw);
    if (full_data) {
      _write_kmall_data(logme.data);
      // Assume that the EMdgmMpartition_def structure will not change so
      // it's safe to collect and write all partitions of this structure even without a
      // compatible structure definition: inform but not try to deserialize until version check
      if(dgmVersion == EMdgm_f::MWC_VERSION){
        //TODO: decode water column ?
      }else if(dgmVersion == EMdgm_h::MWC_VERSION){
        //TODO: decode water column ?
      }else if(dgmVersion == EMdgm_i::MWC_VERSION){
        //TODO: decode water column ?
      }else{
        ROS_ERROR_STREAM("Received MWC unsupported message version "
                         << std::to_string(dgmVersion));
        return false;
      }
    }
  }
  else if (msg_type==EMdgm_f::EM_DGM_C_POSITION){
    r.record_name = "EM_DGM_C_POSITION";
    _write_kmall_data(raw.data);
  }
  else if (msg_type==EMdgm_f::EM_DGM_C_HEAVE){
    r.record_name = "EM_DGM_C_HEAVE";
    _write_kmall_data(raw.data);
  }
  if (r.record_name.empty()){
    ROS_ERROR_STREAM("TYPE : " << msg_type << " not parsed, but recorded anyways");
    r.record_name = "Unknown";
    _write_kmall_data(raw.data);
  }
  d->kmall_record_pub_.publish(r);
  return true;
}

double kongsberg_em::KongsbergEM2040::_timeToLastPartition(const EMdgm_f::EMdgmHeader *hdr)
{
  // Init previous time to now for the first time only (static variable!)
  static ros::Time prev_t = ros::Time::now();

  ros::Time t;
  t.fromSec(hdr->time_sec + hdr->time_nanosec / 1.0e9);
  auto delta_t = t - prev_t;
  prev_t = t;
  // Convert to milliseconds
  return delta_t.toNSec() / 1.0e6;
}

std::pair<bool, ds_core_msgs::RawData>
KongsbergEM2040::check_and_append_mpartition(const ds_core_msgs::RawData& raw_p)
{

  auto ptr = raw_p.data.data();
  auto max_length = raw_p.data.size();
  int count = 0;
  auto hdr = reinterpret_cast<const EMdgm_f::EMdgmHeader *>(ptr + count);
  count += sizeof(EMdgm_f::EMdgmHeader);
  // ROS_ERROR_STREAM("PARTITIONED TIME: "<<hdr->time_sec);

  auto delta_t_ms = _timeToLastPartition(hdr);

  auto partition = reinterpret_cast<const EMdgm_f::EMdgmMpartition *>(ptr + count);
  count += sizeof(EMdgm_f::EMdgmMpartition);
  // If the datagram isn't partitioned, then return itself immediately!
  if (partition->dgmNum == 1 && partition->numOfDgms == 1)
  {
    d->kmall_dgmNum = 0;
    d->kmall_numOfDgms = 0;
    return {true, raw_p};
  }
  // If it's the first in a sequence, then clear out the buffer and resize it to the current size.
  // Set its partition values to 1 and 1 respectively.
  else if (partition->dgmNum == 1 && partition->numOfDgms > 1)
  {
    d->kmall_dgmNum = partition->dgmNum;
    d->kmall_numOfDgms = partition->numOfDgms;
    d->kmall_partitioned.data.resize(max_length);
    d->kmall_partitioned.data = raw_p.data;
    // Rewrite number of Dgms: the reconstructed complete datagram will only have one part
    auto new_part = reinterpret_cast<EMdgm_f::EMdgmMpartition *>(d->kmall_partitioned.data.data() + sizeof(EMdgm_f::EMdgmHeader));
    new_part->dgmNum = 1;
    new_part->numOfDgms = 1;
    d->kmall_dgmNum = 1;
    ROS_DEBUG_STREAM("Part " << d->kmall_dgmNum << "/" << d->kmall_numOfDgms << " with size " << max_length << " DeltaT "
                            << delta_t_ms);
    return {false, {}};
  }
  // If it's a following datagram, then append it starting AFTER the partition.
  else if (partition->dgmNum > 1)
  {
    // Datagrams partitions shall be received sequentially
    auto expected_dgmNum = d->kmall_dgmNum + 1;
    if ((d->kmall_dgmNum + 1) == partition->dgmNum)
    {
      auto current_length = d->kmall_partitioned.data.size();
      if (current_length == 0)
      {
        ROS_ERROR_STREAM("MISSED EARLIER PARTITION PACKET... IGNORING LATER PACKETS");
      }

      if (delta_t_ms > 0)
      {
        ROS_ERROR_STREAM("Packet Timestamp missmatch on reconstruction: dropped !");
        d->kmall_dgmNum = 0;
        d->kmall_numOfDgms = 0;
        return {false, {}};
      }
      // Resize and copy everything except the new header/partition
      // Overwrite the ending values for the length
      d->kmall_partitioned.data.resize(current_length + max_length - count - 4);
      memcpy(d->kmall_partitioned.data.data() + current_length - 4,ptr + count, max_length - count);
      ROS_DEBUG_STREAM("Next Part " << partition->dgmNum<< "/"<< partition->numOfDgms << " with size "<<max_length
                                  <<  " DeltaT " << delta_t_ms);
      d->kmall_dgmNum = partition->dgmNum;
    }else{
      ROS_ERROR_STREAM("MISSED PARTITION PACKET: expected part " << expected_dgmNum
                                                                 << " Received part " << partition->dgmNum
                                                                 << " DeltaT " << delta_t_ms);
      d->kmall_dgmNum = 0;
      d->kmall_numOfDgms = 0;
      return {false, {}};
    }
  }
  // If the datagram has completed transmission, then return the partitioned data message.
  if (partition->dgmNum == partition->numOfDgms){
    auto data_ptr = d->kmall_partitioned.data.data();
    auto data_size = d->kmall_partitioned.data.size();
    auto starting_size_ptr = reinterpret_cast<uint32_t*>(data_ptr);
    *starting_size_ptr = data_size;
    auto ending_size_ptr = reinterpret_cast<uint32_t*>(d->kmall_partitioned.data.data() + d->kmall_partitioned.data.size() - 4);
    *ending_size_ptr = data_size;
    ROS_DEBUG_STREAM("Partition complete! Total size "<< data_size<<" bytes");
    d->kmall_dgmNum = 0;
    d->kmall_numOfDgms = 0;
    return {true, d->kmall_partitioned};
  }
  // Otherwise, assume transmission has not completed and we are waiting for more data.
  return {false, {}};
}

void
KongsbergEM2040::mbraw_to_kmstatus(ds_multibeam_msgs::MultibeamRaw raw)
{
  int num_soundings = raw.beamflag.size();
  int num_good = 0;
  std::vector<float> acrosstrack_angles_raw, ranges, depths;
  float min_range = 100;
  float max_range = 0;
  float min_depth = 100;
  float max_depth = 0;
  float center_angle = 100;
  float center_range = 0;
  float center_depth = 0;
  ranges.resize(num_soundings);
  depths.resize(num_soundings);
  for (int i=0; i<num_soundings; i++){
    if (raw.beamflag[i] != raw.BEAM_BAD_SONAR){
      num_good ++;
    }
    ranges[i] = raw.soundspeed*raw.twowayTravelTime[i] / 2.0;
    depths[i] = ranges[i]*cos(raw.angleAlongTrack[i])*cos(raw.angleAcrossTrack[i]);
    if (abs(raw.angleAcrossTrack[i]) < abs(center_angle)){
      center_angle = raw.angleAcrossTrack[i];
      center_range = ranges[i];
      center_depth = depths[i];
    }
    min_range = (ranges[i] < min_range ? ranges[i] : min_range);
    max_range = (ranges[i] > max_range ? ranges[i] : max_range);
    min_depth = (depths[i] < min_depth ? depths[i] : min_depth);
    max_depth = (depths[i] > max_depth ? depths[i] : max_depth);
  }

  // ping num populated earlier
  d->m_status.num_soundings = num_soundings;
  d->m_status.percent_good = 100.0 * num_good / static_cast<float>(num_soundings);
  d->m_status.min_depth = min_depth;
  d->m_status.max_depth = max_depth;
  d->m_status.center_depth = center_depth;
  d->m_status.min_range = min_range;
  d->m_status.max_range = max_range;
  d->m_status.center_range = center_range;
  d->kmstatus_pub_.publish(d->m_status);
}

void KongsbergEM2040::setupAll()
{
  setupParameters();
  setupPublishers();
  setupServices();
  setupTimers();
}


// XXXXXXXXXX
// Setup Fxns
// ----------
void
KongsbergEM2040::setupServices()
{

  const auto name = ros::this_node::getName();

  std::string ping_srv = ros::param::param<std::string>("~ping_service", "ping_cmd");
  d->ping_srv_ = d->nh_.advertiseService<ds_kongsberg_msgs::PingCmd::Request, ds_kongsberg_msgs::PingCmd::Response>
      (name + "/" + ping_srv, boost::bind(&KongsbergEM2040::_ping_cmd, this, _1, _2));

  std::string power_srv = ros::param::param<std::string>("~power_service", "power_cmd");
  d->power_srv_ = d->nh_.advertiseService<ds_kongsberg_msgs::PowerCmd::Request, ds_kongsberg_msgs::PowerCmd::Response>
      (name + "/" + power_srv, boost::bind(&KongsbergEM2040::_power_cmd, this, _1, _2));
  std::string settings_srv = ros::param::param<std::string>("~settings_service", "settings_cmd");
  d->settings_srv_ = d->nh_.advertiseService<ds_kongsberg_msgs::SettingsCmd::Request,
                                         ds_kongsberg_msgs::SettingsCmd::Response>
      (name + "/" + settings_srv, boost::bind(&KongsbergEM2040::_settings_cmd, this, _1, _2));

  std::string msettings_srv = ros::param::param<std::string>("~multisettings_service", "multi_settings_cmd");
  d->multi_settings_srv_ = d->nh_.advertiseService<ds_kongsberg_msgs::MultiSettingsCmd::Request,
                                                 ds_kongsberg_msgs::MultiSettingsCmd::Response>
      (name + "/" + msettings_srv, boost::bind(&KongsbergEM2040::_multisettings_cmd, this, _1, _2));

  std::string readsettings_srv = ros::param::param<std::string>("~readsettings_service", "read_settings_cmd");
  d->read_settings_srv_ = d->nh_.advertiseService<ds_kongsberg_msgs::SettingsCmd::Request,
          ds_kongsberg_msgs::SettingsCmd::Response>
(name + "/" + readsettings_srv, boost::bind(&KongsbergEM2040::_read_settings_cmd, this, _1, _2));

  std::string bist_srv = ros::param::param<std::string>("~bist_service", "bist_cmd");
  d->bist_srv_ = d->nh_.advertiseService<ds_kongsberg_msgs::BistCmd::Request, ds_kongsberg_msgs::BistCmd::Response>
      (name + "/" + bist_srv, boost::bind(&KongsbergEM2040::_bist_cmd, this, _1, _2));
  std::string xml_srv = ros::param::param<std::string>("~xml_service", "load_xml_cmd");
  d->xml_srv_ = d->nh_.advertiseService<ds_kongsberg_msgs::LoadXmlCmd::Request, ds_kongsberg_msgs::LoadXmlCmd::Response>
      (name + "/" + xml_srv, boost::bind(&KongsbergEM2040::_load_xml_cmd, this, _1, _2));
}
void kongsberg_em::KongsbergEM2040::_reset_rt_params()
{
    d->m_rtParams.rt_force_depth = "0";
    d->m_rtParams.rt_min_depth = "0";
    d->m_rtParams.rt_max_depth = "0";
    d->m_rtParams.rt_ping_freq = "0";
    d->m_rtParams.rt_depth_mode = "0";
    d->m_rtParams.rt_detector_mode = "0";
    d->m_rtParams.rt_fm_disable = "0";
    d->m_rtParams.rt_log_watercol = "0";
    d->m_rtParams.rt_extra_detect = "0";
    d->m_rtParams.rt_pitch_stab = "0";
    d->m_rtParams.rt_tx_angle_along = "0";
    d->m_rtParams.rt_yaw_stab = "0";
    d->m_rtParams.rt_max_ping_rate = "0";
    d->m_rtParams.rt_min_swath_distance = "0";
    d->m_rtParams.rt_trigger = "0";
    d->m_rtParams.rt_port_angle = "0";
    d->m_rtParams.rt_stbd_angle = "0";
    d->m_rtParams.rt_port_coverage = "0";
    d->m_rtParams.rt_stbd_coverage = "0";
}

void kongsberg_em::KongsbergEM2040::_reset_sensor_input_statuses()
{
    d->m_status.cpu_temperature = UNKNOWN_STATUS;
    d->m_status.position_1 =  UNKNOWN_STATUS;
    d->m_status.position_2 =  UNKNOWN_STATUS;
    d->m_status.position_3 = UNKNOWN_STATUS;
    d->m_status.attitude_1 = UNKNOWN_STATUS;
    d->m_status.attitude_2 = UNKNOWN_STATUS;
    d->m_status.depth = UNKNOWN_STATUS;
    d->m_status.svp = UNKNOWN_STATUS;
    d->m_status.time_sync = UNKNOWN_STATUS;
    d->m_status.pps = UNKNOWN_STATUS;
}

void
KongsbergEM2040::setupParameters()
{
  // m_status parameters
  d->m_status = ds_kongsberg_msgs::KongsbergStatus{};
  d->m_status.sounder_name = ros::param::param<std::string>("~sounder_name", "EM2040_40");
  d->m_status.ship_name = ros::param::param<std::string>("~ship_name", "ShipName");
  d->m_status.pu_connected = !ros::param::param<bool>("~run_startup", true);
  d->m_status.bist_directory = ros::param::param<std::string>("~bist_dir", "/tmp/");
  d->m_status.kmall_directory = ros::param::param<std::string>("~kmall_dir", "/tmp/");
  d->m_status.xml_directory = ros::param::param<std::string>("~xml_dir", "/tmp/");
  _reset_sensor_input_statuses();
  _reset_rt_params();
  // other parameters
  d->mrz_frame_id_ = ros::param::param<std::string>("~mrz_frame_id", "sonar_vcs");
  d->save_kmall_files = ros::param::param<bool>("~save_kmall_files", true);
  d->kmall_max_buffer_size = 1e3*ros::param::param<int>("~max_kmall_buffer_kB", 30);
  d->kmall_max_file_size = 1e6*ros::param::param<int>("~max_kmall_file_MB", 400);
  d->kmall_partitioned = ds_core_msgs::RawData{};
  d->kmall_partitioned.data.resize(0);
  _new_kmall_file();
}

void
KongsbergEM2040::setupPublishers()
{

  const auto name = ros::this_node::getName();

  auto mbraw_topic = ros::param::param<std::string>("~mbraw_topic", "mbraw");
  d->mbraw_pub_ = d->nh_.advertise<ds_multibeam_msgs::MultibeamRaw>(name + "/" + mbraw_topic, 1000);

  auto pointcloud_topic = ros::param::param<std::string>("~pointcloud_topic", "pointcloud");
  d->pointcloud_pub_ = d->nh_.advertise<sensor_msgs::PointCloud2>(name + "/" + pointcloud_topic, 1000);

  auto kssis_topic = ros::param::param<std::string>("~kssis_topic", "kssis");
  d->kssis_pub_ = d->nh_.advertise<ds_kongsberg_msgs::KongsbergKSSIS>(name + "/" + kssis_topic, 1000);

  auto offset_topic = ros::param::param<std::string>("~offset_topic", "pu_offset");
  d->offset_pub_ = d->nh_.advertise<ds_core_msgs::ClockOffset>(name + "/" + offset_topic, 1000);

  auto kmall_record_topic = ros::param::param<std::string>("~kmall_records_topic", "kmall_records");
  d->kmall_record_pub_ = d->nh_.advertise<ds_kongsberg_msgs::KongsbergKMAllRecord>(name + "/" + kmall_record_topic, 1000);

  auto kmstatus_topic = ros::param::param<std::string>("~kmstatus_topic", "kmstatus");
  d->kmstatus_pub_ = d->nh_.advertise<ds_kongsberg_msgs::KongsbergStatus>(name + "/" + kmstatus_topic, 1000);

  auto rtparam_topic = ros::param::param<std::string>("~km_rt_params_topic", "runtime_params");
  d->rt_params_pub_ = d->nh_.advertise<ds_kongsberg_msgs::KongsbergRuntimeParams>(name + "/" + rtparam_topic, 1000);

  auto mrz_topic = ros::param::param<std::string>("~mrz_topic", "mrz");
  d->mrz_pub_ = d->nh_.advertise<ds_kongsberg_msgs::KongsbergMRZ>(name + "/" + mrz_topic, 1000);

  auto mrz_full_topic = ros::param::param<std::string>("~mrz_full_topic", "mrz_full");
  d->mrz_full_pub_ = d->nh_.advertise<ds_kongsberg_msgs::KongsbergMRZFull>(name + "/" + mrz_full_topic, 1000);
}

void
KongsbergEM2040::setupTimers()
{

  auto kmall_to = ros::param::param<double>("~kmall_timeout", 3.0);
  d->kmall_timer = d->nh_.createTimer(ros::Duration(kmall_to),
                                  &KongsbergEM2040::_on_kmall_timeout, this);

  auto kctrl_to = ros::param::param<double>("~kctrl_timeout", 3.0);
  d->kctrl_timer = d->nh_.createTimer(ros::Duration(kctrl_to),
                                  &KongsbergEM2040::_on_kctrl_timeout, this);

  auto pu_powered_to = ros::param::param<double>("~pu_powered_timeout", 50.0);
  d->pu_powered_timer = d->nh_.createTimer(ros::Duration(pu_powered_to),
                                       &KongsbergEM2040::_on_pu_powered_timeout, this);

  auto pu_connected_to = ros::param::param<double>("~pu_connected_timeout", 50.0);
  d->pu_connected_timer = d->nh_.createTimer(ros::Duration(pu_connected_to),
                                         &KongsbergEM2040::_on_pu_connected_timeout, this);

  auto pinging_to = ros::param::param<double>("~pinging_timeout", 3.0);
  d->pinging_timer = d->nh_.createTimer(ros::Duration(pinging_to),
                                    &KongsbergEM2040::_on_pinging_timeout, this);

  d->kmall_timer.start();
  d->kctrl_timer.start();
  d->pu_powered_timer.start();
  d->pu_connected_timer.start();
  d->pinging_timer.start();
}

// XXXXXXXXXXXXX
// Service Calls
// -------------
bool
KongsbergEM2040::_ping_cmd(ds_kongsberg_msgs::PingCmd::Request &req, ds_kongsberg_msgs::PingCmd::Response &res)
{

  if (!d->m_status.kctrl_connected){
    res.action += "KCtrl not connected... ";
  }
  if (!d->m_status.pu_powered) {
    res.action += "PU not powered... ";
  }
  if (!d->m_status.pu_connected && req.ping!=req.PING_STARTUP){
    res.action += "PU not connected... ";
  }
  switch (req.ping){
    case ds_kongsberg_msgs::PingCmd::Request::PING_START :
      _send_kctrl_command(SIS_TO_K::LOG_IOP_SVP);
      _send_kctrl_command(SIS_TO_K::START_PING);
      d->m_status.commanded_pinging = true;
      res.action = "Commanded ping start, logged IOP and SVP information";
      break;
    case ds_kongsberg_msgs::PingCmd::Request::PING_STOP :
      // This should stop recording as well
      _send_kctrl_command(SIS_TO_K::STOP_PING);
      d->m_status.commanded_pinging = false;
      res.action = "Commanded ping stop";
      break;
    case ds_kongsberg_msgs::PingCmd::Request::PING_NEWFILE :
      _send_kctrl_command(SIS_TO_K::LOG_IOP_SVP);
      res.action = "Requested IOP SVP Header,  created new kmall file, logged IOP and SVP information";
      break;
    case ds_kongsberg_msgs::PingCmd::Request::PING_STARTUP :
      _startup_sequence();
      res.action = "Commanded startup sequence";
      break;
    case ds_kongsberg_msgs::PingCmd::Request::PING_NEWFILE_FORCE :
      _new_kmall_file();
      res.action = "Forced open newfile";
      break;
    default:
      res.action = "Ping command not recognized";
  }
  return true;
}
bool
KongsbergEM2040::_power_cmd(ds_kongsberg_msgs::PowerCmd::Request &req, ds_kongsberg_msgs::PowerCmd::Response &res)
{
  res.command_sent = _send_kctrl_command(req.power);
  return true;
}
bool
KongsbergEM2040::_settings_cmd(ds_kongsberg_msgs::SettingsCmd::Request &req, ds_kongsberg_msgs::SettingsCmd::Response &res)
{
  res.command_sent = _send_kctrl_param(req.setting_name, req.setting_value);
  return true;
}

bool
KongsbergEM2040::_multisettings_cmd(ds_kongsberg_msgs::MultiSettingsCmd::Request &req, ds_kongsberg_msgs::MultiSettingsCmd::Response &res)
{
  res.command_sent = _send_kctrl_param(req.settings);
  return true;
}

bool
KongsbergEM2040::_read_settings_cmd(ds_kongsberg_msgs::SettingsCmd::Request &req, ds_kongsberg_msgs::SettingsCmd::Response &res)
{
  _read_kctrl_params();
  res.command_sent = true;
  return true;
}

bool
KongsbergEM2040::_bist_cmd(ds_kongsberg_msgs::BistCmd::Request &req, ds_kongsberg_msgs::BistCmd::Response &res)
{

  if (!d->m_status.kctrl_connected
      || !d->m_status.pu_powered
      || !d->m_status.pu_connected){
    ROS_ERROR_STREAM("Sounder not connected, no BIST started.");
    res.action = "Sounder not connected, no BIST started.";
    return true;
  }

  if (d->m_status.bist_running
      && req.bist_command != ds_kongsberg_msgs::BistCmd::Request::BIST_CANCEL){
    res.action = "BIST already in progress... cancel it first";
    return true;
  }
  bist_strings bs;
  std::string bist_name;
  switch (req.bist_command){
    case ds_kongsberg_msgs::BistCmd::Request::BIST_ONDECK :
      d->bist_tests = bs.get_ondeck();
      bist_name = "bist_ondeck";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_INWATER :
      d->bist_tests = bs.get_inwater();
      bist_name = "bist_inwater";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_CANCEL :
      if (d->m_status.bist_running){
        d->m_status.bist_progress = d->bist_tests.size()-1;
        _run_next_bist();
        res.action = "BIST commanded to cancel!";
        return true;
      } else {
        res.action = "No BIST in progress, ready to start a new BIST";
        return true;
      }
    case ds_kongsberg_msgs::BistCmd::Request::BIST_CPU :
      d->bist_tests = std::vector<std::string>({bs.CPU});
      bist_name = "bist_cpu";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_TX_UNIT :
      d->bist_tests = std::vector<std::string>({bs.TX_UNIT});
      bist_name = "bist_tx_unit";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_RX_CHANNELS :
      d->bist_tests = std::vector<std::string>({bs.RX_CHANNELS});
      bist_name = "bist_rx_channels";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_RX_NOISE_SPECTRUM :
      d->bist_tests = std::vector<std::string>({bs.RX_NOISE_SPECTRUM});
      bist_name = "bist_rx_noise_spectrum";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_CBMF :
      d->bist_tests = std::vector<std::string>({bs.CBMF});
      bist_name = "bist_cbmf";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_CBMF_CPU :
      d->bist_tests = std::vector<std::string>({bs.CBMF_CPU});
      bist_name = "bist_cbmf_cpu";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_TX_VIA_RX :
      d->bist_tests = std::vector<std::string>({bs.TX_VIA_RX});
      bist_name = "bist_tx_via_rx";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_SOFTWARE_VERSIONS :
      d->bist_tests = std::vector<std::string>({bs.SOFTWARE_VERSIONS});
      bist_name = "bist_software_versions";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_RX_UNIT :
      d->bist_tests = std::vector<std::string>({bs.RX_UNIT});
      bist_name = "bist_rx_unit";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_RX_CBMF :
      d->bist_tests = std::vector<std::string>({bs.RX_CBMF});
      bist_name = "bist_rx_cbmf";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_RX_NOISE_LEVEL :
      d->bist_tests = std::vector<std::string>({bs.RX_NOISE_LEVEL});
      bist_name = "bist_rx_noise_level";
      break;
    case ds_kongsberg_msgs::BistCmd::Request::BIST_SYSTEM_INFO :
      d->bist_tests = std::vector<std::string>({bs.SYSTEM_INFO});
      bist_name = "bist_system_info";
      break;
    default:
      res.action = "Unknown BIST command... failure!";
      return true;
  }
  d->m_status.bist_count ++;
  d->m_status.bist_running = true;
  d->m_status.bist_progress = -1;
  d->m_status.bist_filename = filename(d->m_status.bist_directory,
                                       d->m_status.bist_count,
                                       bist_name,
                                       ".txt");
  _run_next_bist();
  res.action = "BIST started... check progress in : " + d->m_status.bist_filename;
  return true;
}
bool
KongsbergEM2040::_load_xml_cmd(ds_kongsberg_msgs::LoadXmlCmd::Request &req, ds_kongsberg_msgs::LoadXmlCmd::Response &res)
{
  res.command = _read_kctrl_xml(req.xml_filename);
  return true;
}

// XXXXXXXXXXXXXXXX
// Connection calls
// ----------------
void
KongsbergEM2040::_on_kmall_data(ds_core_msgs::RawData raw)
{
  d->pck_cnt += 1;
  if (!parse_data(raw)){
    ROS_ERROR_STREAM("KMAll data parse failed OR incomplete packet");
  } else {
    d->kmall_timer.stop();
    d->kmall_timer.start();
    d->m_status.kmall_connected = true;
  }
}
void
KongsbergEM2040::_on_kctrl_data(ds_core_msgs::RawData raw)
{

  if (!parse_message(raw)){
    ROS_ERROR_STREAM("KCtrl message parse failed");
  } else {
    d->kctrl_timer.stop();
    d->kctrl_timer.start();
    d->m_status.kctrl_connected = true;
  }
}

// XXXXXXXXXXXXXXXXXXXXXX
// Command send shortcuts
// ----------------------
void
KongsbergEM2040::_startup_sequence()
{

  _send_kctrl_command(SIS_TO_K::START);
  _send_kctrl_command(SIS_TO_K::SET_READY);
  if (d->m_status.commanded_pinging){
    _send_kctrl_command(SIS_TO_K::LOG_IOP_SVP);
    _send_kctrl_command(SIS_TO_K::START_PING);
  }
}
std::string
KongsbergEM2040::_send_kctrl_command(int cmd)
{

  std::stringstream ss;
  ss << "$KSSIS,"
      << cmd << ","
      << d->m_status.sounder_name;
  auto msg = ss.str();
  d->send_kctrl_data_(msg);
  return msg;
}

template <class T1>
std::string
KongsbergEM2040::_send_kctrl_param(std::string param, T1 val)
{
  return _send_kctrl_param(std::vector<std::string>{param}, std::vector<std::string>{val});
}

template <class T1>
std::string
KongsbergEM2040::_send_kctrl_param(std::vector<std::string> params, std::vector<T1> vals)
{

  std::stringstream ss;
  ss << "$KSSIS,"
     << SIS_TO_K::SETVALUES << ","
     << d->m_status.sounder_name;
  for (size_t i=0; i<params.size(); i++){
    ss << ","
       << params[i] << "="
       << vals[i];
  }
  auto msg = ss.str();
  d->send_kctrl_data_(msg);
  return msg;
}

std::string
KongsbergEM2040::_send_kctrl_param(std::vector<KSetting> params)
{

  std::stringstream ss;
  ss << "$KSSIS,"
     << SIS_TO_K::SETVALUES << ","
     << d->m_status.sounder_name;
  for (size_t i=0; i<params.size(); i++){
    ss << ","
       << params[i].name << "="
       << params[i].value;
  }
  auto msg = ss.str();
  d->send_kctrl_data_(msg);
  return msg;
}
void
KongsbergEM2040::_read_kctrl_params()
{

  std::stringstream ss;
  ss << "$KSSIS,"
     << SIS_TO_K::GETVALUES << ","
     << d->m_status.sounder_name;
  auto msg = ss.str();
  d->send_kctrl_data_(msg);
}

void
KongsbergEM2040::_print_bist(std::string name, std::string status, std::string msg)
{

  if (!d->m_status.bist_running){
    return;
  }
  d->bist_summary_stream << status << "\t" << name << "\n";
  std::ofstream fs;
  fs.open (d->m_status.bist_filename, std::ios::app);
  fs << "\n";
  fs << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n";
  fs << name << "\t" << status << "\n";
  fs << "--------------------------------\n";
  fs << msg;
  fs.close();
}

void
KongsbergEM2040::_run_next_bist()
{

  if (!d->m_status.bist_running){
    return;
  }
  d->m_status.bist_progress++;
  if (d->m_status.bist_progress == 0){
    d->bist_summary_stream.str("");
    std::ofstream fs;
    fs.open (d->m_status.bist_filename, std::ios::app);
    fs << "BIST started at " << boost::posix_time::second_clock::universal_time() << "\n";
    fs << "Includes tests...\n";
    for (const auto& test : d->bist_tests){
      fs << "\t" << test << "\n";
    }
    fs << "--------------------------------\n";
    fs.close();
  }
  if (d->m_status.bist_progress < d->bist_tests.size() ){
    ROS_INFO_STREAM("Running BIST ... "<< d->bist_tests[d->m_status.bist_progress]);
    d->m_status.bist_current = d->bist_tests[d->m_status.bist_progress];
    // Send the next command
    _send_kctrl_param("INST_PARAM_BIST_DO", d->bist_tests[d->m_status.bist_progress]);
  } else if (d->m_status.bist_progress >= d->bist_tests.size()){
    std::ofstream fs;
    fs.open (d->m_status.bist_filename, std::ios::app);
    fs << "\nXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n\n";
    fs << "BIST completed at " << boost::posix_time::second_clock::universal_time() << "\n";
    fs << "Ran tests...\n";
    fs << d->bist_summary_stream.str();
    fs.close();
    ROS_INFO_STREAM("BIST done! \n"<<d->bist_summary_stream.str()<<"Full results in "<<d->m_status.bist_filename);
    d->bist_tests.resize(0);
    d->m_status.bist_running = false;
    d->m_status.bist_current = "";
  }
}
void
KongsbergEM2040::_new_kmall_file()
{
  if (not d->save_kmall_files)
  {
    return;
  }

  d->m_status.kmall_filecount ++;
  d->m_status.kmall_filename = filename(d->m_status.kmall_directory,
                                        d->m_status.kmall_filecount,
                                        d->m_status.ship_name,
                                        ".kmall");
  if (d->kmall_stream.is_open()){
    d->kmall_stream.close();
  }
  d->kmall_stream.open (d->m_status.kmall_filename, std::ios::out | std::ios::binary);
  d->kmall_buffer_size = 0;
  d->kmall_file_size = 0;
  d->m_status.kmall_filesize_kB = 0;
  ROS_WARN_STREAM("New kmall file: " << d->m_status.kmall_filename);
}
void
KongsbergEM2040::_write_kmall_data(const std::vector<uint8_t>& data)
{

  if (d->kmall_stream.is_open()){
    auto size = data.size();
    auto bytes = reinterpret_cast<const char*>(data.data());
    d->kmall_stream.write(bytes, size);
    d->kmall_buffer_size += size;
    d->kmall_file_size += size;
    d->m_status.kmall_filesize_kB += d->kmall_file_size / 1e3;
    if (d->kmall_file_size > d->kmall_max_file_size){
      _send_kctrl_command(SIS_TO_K::LOG_IOP_SVP);
    } else if (d->kmall_buffer_size > d->kmall_max_buffer_size){
      d->kmall_stream.flush();
      d->kmall_buffer_size = 0;
    }
  }
}

void
KongsbergEM2040::_write_kctrl_xml(std::vector<uint8_t>& data)
{

  auto xml_string = std::string{ reinterpret_cast<const char*>(data.data()), data.size() };
  d->m_status.xml_filecount++;
  d->m_status.xml_filename = filename(d->m_status.xml_directory,
                                      d->m_status.xml_filecount,
                                      d->m_status.sounder_name,
                                      ".xml");
  std::ofstream fs;
  fs.open (d->m_status.xml_filename, std::ios::binary);
  fs << xml_string;
  fs.close();
  ROS_INFO_STREAM("Logged XML in "<<d->m_status.xml_filename);
  // Now look for some important key-value pairs
  std::vector<std::string> params, vals;
  std::tie(params, vals) = string_split_out_xml_params(xml_string);
  if (params.size() != vals.size()){
    ROS_ERROR_STREAM("Bad XML received, logged anyways");
    return;
  }
  for (size_t i=0; i<params.size(); i++){
    if (params[i] == "SDPM1") {
      ROS_INFO_STREAM("PING FREQ: " << params[i] << " val: " << vals[i]);
      d->m_rtParams.rt_ping_freq = vals[i];
    } else if (params[i] == "SDFD") {
      ROS_INFO_STREAM("FORCE DEPTH param: "<<params[i]<<" val: "<<vals[i]);
      d->m_rtParams.rt_force_depth = vals[i];
    } else if (params[i] == "SDMA") {
      ROS_INFO_STREAM("MAX DEPTH param: "<<params[i]<<" val: "<<vals[i]);
      d->m_rtParams.rt_max_depth = vals[i];
    } else if (params[i] == "SDMI") {
      ROS_INFO_STREAM("MIN DEPTH param: " << params[i] << " val: " << vals[i]);
      d->m_rtParams.rt_min_depth = vals[i];
    } else if (params[i] == "SDDM") {
      ROS_INFO_STREAM("DETECTOR MODE param: "<<params[i]<<" val: "<<vals[i]);
      d->m_rtParams.rt_detector_mode = vals[i];
    } else if (params[i] == "SDFM") {
      ROS_INFO_STREAM("FM DISABLE param: " << params[i] << " val: " << vals[i]);
      d->m_rtParams.rt_fm_disable = vals[i];
    } else if (params[i] == "SDED") {
      ROS_INFO_STREAM("EXTRA DETECT param: "<<params[i]<<" val: "<<vals[i]);
      d->m_rtParams.rt_extra_detect = vals[i];
    } else if (params[i] == "SDPT1") {
      ROS_INFO_STREAM("DEPTH MODE param: "<<params[i]<<" val: "<<vals[i]);
      d->m_rtParams.rt_depth_mode = vals[i];
    } else if (params[i] == "STET") {
      ROS_INFO_STREAM("TRIGGER param: "<<params[i]<<" val: "<<vals[i]);
      d->m_rtParams.rt_trigger = vals[i];
    } else if (params[i] == "STYAMO") {
      ROS_INFO_STREAM("YAW STAB param: "<<params[i]<<" val: "<<vals[i]);
      d->m_rtParams.rt_yaw_stab = vals[i];
    } else if (params[i] == "STXA") {
      ROS_INFO_STREAM("TX ANGLE param: " << params[i] << " val: " << vals[i]);
      d->m_rtParams.rt_tx_angle_along = vals[i];
    } else if (params[i] == "STPS") {
      ROS_INFO_STREAM("PITCH STAB param: " << params[i] << " val: " << vals[i]);
      d->m_rtParams.rt_pitch_stab = vals[i];
    } else if (params[i] == "STPF") {
      ROS_INFO_STREAM("PING RATE param: " << params[i] << " val: " << vals[i]);
      d->m_rtParams.rt_max_ping_rate = vals[i];
    } else if (params[i] == "STPK") {
      ROS_INFO_STREAM("MIN SWATH param: " << params[i] << " val: " << vals[i]);
      d->m_rtParams.rt_min_swath_distance = vals[i];
    } else if(params[i] == "SSPA1"){
      ROS_INFO_STREAM("MAX PORT ANGLE param: " << params[i] << " val: " << vals[i]);
      d->m_rtParams.rt_port_angle = vals[i];
    } else if (params[i] == "SSSA1"){
      ROS_INFO_STREAM("MAX STBD ANGLE param: " << params[i] << " val: " << vals[i]);
      d->m_rtParams.rt_stbd_angle = vals[i];
    } else if (params[i] == "SSPC"){
      ROS_INFO_STREAM("PORT COVERAGE param: " << params[i] << " val: " << vals[i]);
      d->m_rtParams.rt_port_coverage = vals[i];
    } else if (params[i] == "SSSC"){
      ROS_INFO_STREAM("STBD COVERAGE param: " << params[i] << " val: " << vals[i]);
      d->m_rtParams.rt_stbd_coverage = vals[i];
    }

  }
  d->rt_params_pub_.publish(d->m_rtParams);
}

std::string
KongsbergEM2040::_read_kctrl_xml(std::string filename)
{

  std::vector<std::string> params, vals;
  std::tie(params, vals) = file_split_out_xml_params(filename);
  ROS_ERROR_STREAM(params.size() << " params found, " << vals.size() << " vals found");
  if (params.size() != vals.size()){
    return "No command sent, params and vals don't match";
  }
  return _send_kctrl_param(params, vals);
}

// XXXXXXXXXXXXXXX
// Status timeouts
// ---------------

void
KongsbergEM2040::_on_kctrl_timeout(const ros::TimerEvent&)
{
  if (d->m_status.kctrl_connected){
    ROS_ERROR_STREAM("Kctrl timed out... assume totally disconnected");
  }
  d->m_status.kctrl_connected = false;
  d->m_status.pu_powered = false;
  d->m_status.pu_connected = false;
  d->m_status.bist_running = false;
  _reset_sensor_input_statuses();
  d->kmstatus_pub_.publish(d->m_status);
}

void
KongsbergEM2040::_on_pu_powered_timeout(const ros::TimerEvent&)
{


  if (d->m_status.pu_powered){
    ROS_ERROR_STREAM("PU power timed out... assume powered off");
  }
  d->m_status.pu_powered = false;
  d->m_status.pu_connected = false;
  d->m_status.bist_running = false;
  d->kmstatus_pub_.publish(d->m_status);
}

void
KongsbergEM2040::_on_pu_connected_timeout(const ros::TimerEvent&)
{


  if (d->m_status.pu_connected){
    ROS_ERROR_STREAM("PU connection timed out... assume disconnected");
  }
  d->m_status.pu_connected = false;
  d->m_status.bist_running = false;
  d->kmstatus_pub_.publish(d->m_status);
}

void
KongsbergEM2040::_on_kmall_timeout(const ros::TimerEvent&)
{


  if (d->m_status.kmall_connected){
    ROS_ERROR_STREAM("Kmall stream timed out... assume totally disconnected");
    ROS_DEBUG_STREAM("Received " << d->pck_cnt << " KMALL packets");
    d->pck_cnt = 0;
  }
  d->m_status.kmall_connected = false;
  d->m_status.pinging = false;
  d->kmstatus_pub_.publish(d->m_status);
}

void
KongsbergEM2040::_on_pinging_timeout(const ros::TimerEvent&)
{


  if(d->m_status.commanded_pinging && d->m_status.kmall_connected){
    ROS_ERROR_STREAM("Ping timed out... attempt to restart pinging");
    _send_kctrl_command(SIS_TO_K::LOG_IOP_SVP);
    _send_kctrl_command(SIS_TO_K::START_PING);
  }
  d->m_status.pinging = false;
  d->kmstatus_pub_.publish(d->m_status);
}

template <class EMdgmMRZ_S>
void KongsbergEM2040::_read_and_publish_mrz(const ds_kongsberg_msgs::KongsbergKMAllRecord &r,
                                            std::vector<uint8_t> &data)
{
  EMdgmMRZ_S mrz;
  bool ok = false;
  std::tie(ok, mrz) = kmall::read_mrz<EMdgmMRZ_S>(data.data(),data.size());
  if (ok){
    auto mbr = mrz_to_mb_raw(&mrz);
    mbr.header = r.header;
    mbr.ds_header = r.ds_header;
    d->mbraw_pub_.publish(mbr);

    auto mrz_msg = mrz_to_msg(mrz);
    mrz_msg.header = r.header;
    mrz_msg.ds_header = r.ds_header;
    d->mrz_pub_.publish(mrz_msg);

    auto mrz_full_msg = mrz_to_msg_full(mrz);
    mrz_full_msg.header = r.header;
    d->mrz_full_pub_.publish(mrz_full_msg);

    d->pointcloud_pub_.publish(mrz_to_pointcloud2(mrz, d->mrz_frame_id_));
    auto delta_ping = mrz.cmnPart.pingCnt - d->m_status.ping_num;
    ROS_ERROR_STREAM_COND((delta_ping > 1) && (d->m_status.ping_num != 0),
                          "Missed ping between " << d->m_status.ping_num << " and " << mrz.cmnPart.pingCnt);

    d->m_status.ping_num = mrz.cmnPart.pingCnt;
    mbraw_to_kmstatus(mbr);
  }
  else
  {
    d->m_decodingFailed = true;
  }
}

} //namespace
