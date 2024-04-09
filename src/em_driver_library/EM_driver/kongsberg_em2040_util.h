  //
// Created by jvaccaro on 7/17/19.
//

#pragma once

#define M_PI_RAD 3.14159
#define M_PI_DEG 180.0

#include "boost/date_time/posix_time/posix_time_io.hpp"
#include "boost/date_time/posix_time/posix_time_types.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <ds_multibeam_msgs/MultibeamRaw.h>
#include <ds_kongsberg_msgs/KongsbergMRZ.h>

namespace kongsberg_em {

std::string filename(std::string directory, int count, std::string base, std::string extension)
{
  std::stringstream filename_ss;
  auto facet = new boost::posix_time::time_facet("%Y%m%d_%H%M");
  filename_ss.imbue(std::locale(filename_ss.getloc(), facet));
  filename_ss << directory;
  filename_ss << std::setfill('0') << std::setw(4) << count;
  filename_ss << "_" << boost::posix_time::second_clock::universal_time();
  filename_ss << "_" << base;
  filename_ss << extension;
  return filename_ss.str();
}

double deg_to_rad(double deg){
  return deg*M_PI_RAD/M_PI_DEG;
}

double rad_to_deg(double rad){
  return rad*M_PI_DEG/M_PI_RAD;
}

std::pair<std::vector<std::string>, std::vector<std::string>>
string_split_out_xml_params(std::string data, std::string token_param="ID", std::string token_value="VALUE", std::string token_delim="TOKEN")
{
  std::vector<std::string> params, values;
  std::stringstream ss(data);
  std::string l;
  std::string param_start = "<" + token_param + ">";
  std::string param_end = "</" + token_param + ">";
  std::string value_start = "<" + token_value + ">";
  std::string value_end = "</" + token_value + ">";
  std::string delim_start = "<" + token_delim + ">";
  std::string delim_end = "</" + token_delim + ">";
  std::string param{};
  std::string value{};
  while(std::getline(ss >> std::ws, l,'\n')){
    if (!l.find(param_start)){
      param = l.substr(param_start.length(), l.length() - param_start.size() - param_end.length());
    } else if (!l.find(value_start)) {
      value = l.substr(value_start.length(), l.length() - value_start.size() - value_end.length());
    } else if (!l.find(delim_start)){
      param = "";
      value = "";
    } else if (!l.find(delim_end)){
      if (!param.empty() && !value.empty()){
        params.push_back(param);
        values.push_back(value);
      }
    }
  }
  return {params, values};
}

std::pair<std::vector<std::string>, std::vector<std::string>>
file_split_out_xml_params(std::string filename, std::string token_param="ID", std::string token_value="VALUE", std::string token_delim="TOKEN")
{
  std::vector<std::string> params, values;
  std::string param_start = "<" + token_param + ">";
  std::string param_end = "</" + token_param + ">";
  std::string value_start = "<" + token_value + ">";
  std::string value_end = "</" + token_value + ">";
  std::string delim_start = "<" + token_delim + ">";
  std::string delim_end = "</" + token_delim + ">";
  std::string param{};
  std::string value{};
  std::string l;
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  while(in >> l){
    if (!l.find(param_start)){
      param = l.substr(param_start.length(), l.length() - param_start.size() - param_end.length());
    } else if (!l.find(value_start)) {
      value = l.substr(value_start.length(), l.length() - value_start.size() - value_end.length());
    } else if (!l.find(delim_start)){
      param = "";
      value = "";
    } else if (!l.find(delim_end)){
      if (!param.empty() && !value.empty()){
        params.push_back(param);
        values.push_back(value);
      }
    }
  }
  return {params, values};
}

template <typename EMdgmMRZ_S>
static ds_kongsberg_msgs::KongsbergMRZ mrz_to_msg(const EMdgmMRZ_S& msg)
{
  ds_kongsberg_msgs::KongsbergMRZ m;
  ros::Time t;
  m.header.stamp = t.fromSec(msg.header.time_sec + msg.header.time_nanosec / 1.0e9);
  m.latitude_deg = msg.pingInfo.latitude_deg;
  m.longitude_deg  = msg.pingInfo.longitude_deg;
  m.ellipsoidHeightReRefPoint_m  = msg.pingInfo.ellipsoidHeightReRefPoint_m;
  m.headingVessel_deg  = msg.pingInfo.headingVessel_deg;
  m.z_waterLevelReRefPoint_m  = msg.pingInfo.z_waterLevelReRefPoint_m;
  return m;
}

template <typename EMdgmMRZ_S>
static ds_multibeam_msgs::MultibeamRaw mrz_to_mb_raw(EMdgmMRZ_S* msg){
  ds_multibeam_msgs::MultibeamRaw mb{};
  ros::Time t;
  mb.header.stamp = t.fromSec(msg->header.time_sec + msg->header.time_nanosec / 1.0e9);

  // Only take bottom soundings: extra detections are from the water column and shall be ignored
  int num_soundings = msg->rxInfo.numSoundingsMaxMain;
  mb.beamflag.resize(num_soundings);
  mb.twowayTravelTime.resize(num_soundings);
  mb.txDelay.resize(num_soundings);
  mb.intensity.resize(num_soundings);
  mb.angleAlongTrack.resize(num_soundings);
  mb.angleAcrossTrack.resize(num_soundings);
  mb.beamwidthAlongTrack.resize(num_soundings);
  mb.beamwidthAcrossTrack.resize(num_soundings);
  for (int i = 0; i < num_soundings; i++) {
    mb.beamflag[i] = (msg->sounding[i].detectionType == 2 ? mb.BEAM_BAD_SONAR : mb.BEAM_OK);
    mb.twowayTravelTime[i] = msg->sounding[i].twoWayTravelTime_sec;
    mb.txDelay[i] = msg->sounding[i].twoWayTravelTimeCorrection_sec;
    mb.intensity[i] = msg->sounding[i].reflectivity1_dB;
    int sector = msg->sounding[i].txSectorNumb;
    if (sector < msg->pingInfo.numTxSectors){
      mb.angleAlongTrack[i] = deg_to_rad(msg->sectorInfo[sector].tiltAngleReTx_deg); // use sector index to get tilt angle, then convert to rad
    }
    mb.angleAcrossTrack[i] = deg_to_rad(msg->sounding[i].beamAngleReRx_deg); // convert deg to rad
    mb.beamwidthAlongTrack[i] = 0;
    mb.beamwidthAcrossTrack[i] = deg_to_rad(msg->sounding[i].WCNomBeamAngleAcross_deg);
  }

  mb.soundspeed = msg->pingInfo.soundSpeedAtTxDepth_mPerSec;
  return mb;
}

template <typename EMdgmMRZ_S>
static sensor_msgs::PointCloud2 mrz_to_pointcloud2(const EMdgmMRZ_S& msg,
                                                  const std::string &frame_id){
  pcl::PointCloud<pcl::PointXYZI> pcl;

  // Only take bottom soundings: extra detections are from the water column and shall be ignored
  int num_soundings = msg.rxInfo.numSoundingsMaxMain;
  pcl::PointXYZI pt;
  for (int i = 0; i < num_soundings; i++)
  {
    pt.x = msg.sounding[i].x_reRefPoint_m;
    pt.y = msg.sounding[i].y_reRefPoint_m;
    pt.z = msg.sounding[i].z_reRefPoint_m - msg.pingInfo.z_waterLevelReRefPoint_m;
    pt.intensity = msg.sounding[i].reflectivity1_dB;
    pcl.push_back(pt);
  }

  sensor_msgs::PointCloud2 m;
  pcl::toROSMsg(pcl, m);
  m.header.stamp = ros::Time().fromSec(msg.header.time_sec + msg.header.time_nanosec / 1.0e9);
  m.header.frame_id = frame_id;
  return m;
}


} //namespace
