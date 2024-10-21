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
#include <ds_kongsberg_msgs/KongsbergMRZFull.h>

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
static ds_kongsberg_msgs::KongsbergMRZFull mrz_to_msg_full(const EMdgmMRZ_S& msg)
{
  ds_kongsberg_msgs::KongsbergMRZFull m;
  ros::Time t;
  m.header.stamp = t.fromSec(msg.header.time_sec + msg.header.time_nanosec / 1.0e9);
  m.dgmVersion = msg.header.dgmVersion;
  m.systemID = msg.header.systemID;
  m.echoSounderID = msg.header.echoSounderID;
  
  m.numOfDgms = msg.partition.numOfDgms;
  m.dgmNum = msg.partition.dgmNum;
  
  m.pingCnt = msg.cmnPart.pingCnt;
  m.rxFansPerPing = msg.cmnPart.rxFansPerPing; 
  m.rxFanIndex = msg.cmnPart.rxFanIndex;
  m.swathsPerPing = msg.cmnPart.swathsPerPing;
  m.swathAlongPosition = msg.cmnPart.swathAlongPosition;
  m.txTransducerInd = msg.cmnPart.txTransducerInd;
  m.rxTransducerInd = msg.cmnPart.rxTransducerInd;
  m.numRxTransducers = msg.cmnPart.numRxTransducers;
  m.algorithmType = msg.cmnPart.algorithmType;

  m.pingRate_Hz = msg.pingInfo.pingRate_Hz;
  m.beamSpacing = msg.pingInfo.beamSpacing;
  m.depthMode = msg.pingInfo.depthMode;
  m.subDepthMode = msg.pingInfo.subDepthMode;
  m.distanceBtwSwath = msg.pingInfo.distanceBtwSwath;
  m.detectionMode = msg.pingInfo.detectionMode;
  m.pulseForm = msg.pingInfo.pulseForm;

  m.frequencyMode_Hz = msg.pingInfo.frequencyMode_Hz;
  m.freqRangeLowLim_Hz = msg.pingInfo.freqRangeLowLim_Hz;
  m.freqRangeHighLim_Hz = msg.pingInfo.freqRangeHighLim_Hz;
  m.maxTotalTxPulseLength_sec = msg.pingInfo.maxTotalTxPulseLength_sec;
  m.maxEffTxPulseLength_sec = msg.pingInfo.maxEffTxPulseLength_sec;
  m.maxEffTxBandWidth_Hz = msg.pingInfo.maxEffTxBandWidth_Hz;
  m.absCoeff_dBPerkm = msg.pingInfo.absCoeff_dBPerkm;
  m.portSectorEdge_deg = msg.pingInfo.portSectorEdge_deg;
  m.starbSectorEdge_deg = msg.pingInfo.starbSectorEdge_deg;
  m.portMeanCov_deg = msg.pingInfo.portMeanCov_deg;
  m.starbMeanCov_deg = msg.pingInfo.starbMeanCov_deg;
  m.portMeanCov_m = msg.pingInfo.portMeanCov_m;
  m.starbMeanCov_m = msg.pingInfo.starbMeanCov_m;
  m.modeAndStabilisation = msg.pingInfo.modeAndStabilisation;
  m.runtimeFilter1 = msg.pingInfo.runtimeFilter1;
  m.runtimeFilter2 = msg.pingInfo.runtimeFilter2;
  m.pipeTrackingStatus = msg.pingInfo.pipeTrackingStatus;
  m.transmitArraySizeUsed_deg = msg.pingInfo.transmitArraySizeUsed_deg;
  m.receiveArraySizeUsed_deg = msg.pingInfo.receiveArraySizeUsed_deg;
  m.transmitPower_dB = msg.pingInfo.transmitPower_dB;
  m.SLrampUpTimeRemaining = msg.pingInfo.SLrampUpTimeRemaining;

  m.yawAngle_deg = msg.pingInfo.yawAngle_deg;
  m.numTxSectors = msg.pingInfo.numTxSectors;
  m.numBytesPerTxSector = msg.pingInfo.numBytesPerTxSector;
  m.headingVessel_deg = msg.pingInfo.headingVessel_deg;
  m.soundSpeedAtTxDepth_mPerSec = msg.pingInfo.soundSpeedAtTxDepth_mPerSec;
  m.txTransducerDepth_m = msg.pingInfo.txTransducerDepth_m;
  m.z_waterLevelReRefPoint_m = msg.pingInfo.z_waterLevelReRefPoint_m;
  m.x_kmallToall_m = msg.pingInfo.x_kmallToall_m;
  m.y_kmallToall_m = msg.pingInfo.y_kmallToall_m;
  m.latLongInfo = msg.pingInfo.latLongInfo;
  m.posSensorStatus = msg.pingInfo.posSensorStatus;
  m.attitudeSensorStatus = msg.pingInfo.attitudeSensorStatus;
  m.latitude_deg = msg.pingInfo.latitude_deg;
  m.longitude_deg = msg.pingInfo.longitude_deg;
  m.ellipsoidHeightReRefPoint_m = msg.pingInfo.ellipsoidHeightReRefPoint_m;

  for(int i = 0; i < msg.pingInfo.numTxSectors; i++)
  {
    ds_kongsberg_msgs::KongsbergMRZtxSectorInfo sectorInfo;
    sectorInfo.txSectorNumb = msg.sectorInfo[i].txSectorNumb;
    sectorInfo.txArrNumber = msg.sectorInfo[i].txArrNumber;
    sectorInfo.txSubArray = msg.sectorInfo[i].txSubArray;
    sectorInfo.sectorTransmitDelay_sec = msg.sectorInfo[i].sectorTransmitDelay_sec;
    sectorInfo.tiltAngleReTx_deg = msg.sectorInfo[i].tiltAngleReTx_deg;
    sectorInfo.txNominalSourceLevel_dB = msg.sectorInfo[i].txNominalSourceLevel_dB;
    sectorInfo.txFocusRange_m = msg.sectorInfo[i].txFocusRange_m;
    sectorInfo.centreFreq_Hz = msg.sectorInfo[i].centreFreq_Hz;
    sectorInfo.signalBandWidth_Hz = msg.sectorInfo[i].signalBandWidth_Hz;
    sectorInfo.totalSignalLength_sec = msg.sectorInfo[i].totalSignalLength_sec;
    sectorInfo.pulseShading = msg.sectorInfo[i].pulseShading;
    sectorInfo.signalWaveForm = msg.sectorInfo[i].signalWaveForm;
    m.sectorInfo.push_back(sectorInfo);
  }

  m.numBytesRxInfo = msg.rxInfo.numBytesRxInfo;
  m.numSoundingsMaxMain = msg.rxInfo.numSoundingsMaxMain;
  m.numSoundingsValidMain = msg.rxInfo.numSoundingsValidMain;
  m.numBytesPerSounding = msg.rxInfo.numBytesPerSounding;
  m.WCSampleRate = msg.rxInfo.WCSampleRate;
  m.seabedImageSampleRate = msg.rxInfo.seabedImageSampleRate;
  m.BSnormal_dB = msg.rxInfo.BSnormal_dB;
  m.BSoblique_dB = msg.rxInfo.BSoblique_dB;
  m.extraDetectionAlarmFlag = msg.rxInfo.extraDetectionAlarmFlag;
  m.numExtraDetections = msg.rxInfo.numExtraDetections;
  m.numExtraDetectionClasses = msg.rxInfo.numExtraDetectionClasses;
  m.numBytesPerClass = msg.rxInfo.numBytesPerClass;

  std::size_t number_of_ss_samples = 0;
  for(int i = 0; i < msg.rxInfo.numSoundingsMaxMain + msg.rxInfo.numExtraDetections; i++)
  {
    ds_kongsberg_msgs::KongsbergMRZsounding sounding;
    sounding.soundingIndex = msg.sounding[i].soundingIndex;
    sounding.txSectorNumb = msg.sounding[i].txSectorNumb;
    sounding.detectionType = msg.sounding[i].detectionType;
    sounding.detectionMethod = msg.sounding[i].detectionMethod;
    sounding.rejectionInfo1 = msg.sounding[i].rejectionInfo1;
    sounding.rejectionInfo2 = msg.sounding[i].rejectionInfo2;
    sounding.postProcessingInfo = msg.sounding[i].postProcessingInfo;
    sounding.detectionClass = msg.sounding[i].detectionClass;
    sounding.detectionConfidenceLevel = msg.sounding[i].detectionConfidenceLevel;

    sounding.rangeFactor = msg.sounding[i].rangeFactor;
    sounding.qualityFactor = msg.sounding[i].qualityFactor;
    sounding.detectionUncertaintyVer_m = msg.sounding[i].detectionUncertaintyVer_m;
    sounding.detectionUncertaintyHor_m = msg.sounding[i].detectionUncertaintyHor_m;
    sounding.detectionWindowLength_sec = msg.sounding[i].detectionWindowLength_sec;
    sounding.echoLength_sec = msg.sounding[i].echoLength_sec;
    sounding.WCBeamNumb = msg.sounding[i].WCBeamNumb;
    sounding.WCrange_samples = msg.sounding[i].WCrange_samples;
    sounding.WCNomBeamAngleAcross_deg = msg.sounding[i].WCNomBeamAngleAcross_deg;
    sounding.meanAbsCoeff_dBPerkm = msg.sounding[i].meanAbsCoeff_dBPerkm;
    sounding.reflectivity1_dB = msg.sounding[i].reflectivity1_dB;
    sounding.reflectivity2_dB = msg.sounding[i].reflectivity2_dB;
    sounding.receiverSensitivityApplied_dB = msg.sounding[i].receiverSensitivityApplied_dB;
    sounding.sourceLevelApplied_dB = msg.sounding[i].sourceLevelApplied_dB;
    sounding.BScalibration_dB = msg.sounding[i].BScalibration_dB;
    sounding.TVG_dB = msg.sounding[i].TVG_dB;
    sounding.beamAngleReRx_deg = msg.sounding[i].beamAngleReRx_deg;
    sounding.beamAngleCorrection_deg = msg.sounding[i].beamAngleCorrection_deg;
    sounding.twoWayTravelTime_sec = msg.sounding[i].twoWayTravelTime_sec;
    sounding.twoWayTravelTimeCorrection_sec = msg.sounding[i].twoWayTravelTimeCorrection_sec;
    sounding.deltaLatitude_deg = msg.sounding[i].deltaLatitude_deg;
    sounding.deltaLongitude_deg = msg.sounding[i].deltaLongitude_deg;
    sounding.z_reRefPoint_m = msg.sounding[i].z_reRefPoint_m;
    sounding.y_reRefPoint_m = msg.sounding[i].y_reRefPoint_m;
    sounding.x_reRefPoint_m = msg.sounding[i].x_reRefPoint_m;
    sounding.beamIncAngleAdj_deg = msg.sounding[i].beamIncAngleAdj_deg;
    sounding.realTimeCleanInfo = msg.sounding[i].realTimeCleanInfo;
    sounding.SIstartRange_samples = msg.sounding[i].SIstartRange_samples;
    sounding.SIcentreSample = msg.sounding[i].SIcentreSample;
    sounding.SInumSamples = msg.sounding[i].SInumSamples;
    m.soundings.push_back(sounding);
    number_of_ss_samples += sounding.SInumSamples;
  }

  for(std::size_t i = 0; i < number_of_ss_samples; i++)
  {
    m.SIsample_desidB.push_back(msg.SIsample_desidB[i]);
  }

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
    // detectionType
    //   0 = normal detection
    //   1 = extra detection
    //   2 = rejected detection In case 2, the estimated range has been used to fill in
    //       amplitude samples in the seabed image datagram.
    //
    // detectionMethod
    //   0 = no valid detection
    //   1 = amplitude detection
    //   2 = phase detection
    //   3-15 for future use.
    if(msg.sounding[i].detectionType == 0 && msg.sounding[i].detectionMethod != 0)
    {
      pt.x = msg.sounding[i].x_reRefPoint_m;
      pt.y = msg.sounding[i].y_reRefPoint_m;
      pt.z = msg.sounding[i].z_reRefPoint_m - msg.pingInfo.z_waterLevelReRefPoint_m;
      pt.intensity = msg.sounding[i].reflectivity1_dB;
      pcl.push_back(pt);
    }
  }

  sensor_msgs::PointCloud2 m;
  pcl::toROSMsg(pcl, m);
  m.header.stamp = ros::Time().fromSec(msg.header.time_sec + msg.header.time_nanosec / 1.0e9);
  m.header.frame_id = frame_id;
  return m;
}


} //namespace
