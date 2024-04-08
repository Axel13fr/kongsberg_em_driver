#pragma once

#include "EMdgmFormat.h"
#include "EMdgmFormat_h.h"
#include "EMdgmFormat_i.h"
#include <utility>
#include <cstring>
#include <ros/console.h>

namespace kmall
{
// The code below assumes manu constants and structures not to change between protocol revisions
// This is checked at compile time here
static_assert(EMdgm_f::MAX_NUM_BEAMS == EMdgm_h::MAX_NUM_BEAMS
              && EMdgm_i::MAX_NUM_BEAMS == EMdgm_h::MAX_NUM_BEAMS, "Format definition missmatch");
static_assert(EMdgm_f::MAX_NUM_BEAMS == EMdgm_h::MAX_NUM_BEAMS
              && EMdgm_i::MAX_NUM_BEAMS == EMdgm_h::MAX_NUM_BEAMS, "Format definition missmatch");
static_assert(EMdgm_f::MAX_EXTRA_DET == EMdgm_h::MAX_EXTRA_DET
              && EMdgm_i::MAX_EXTRA_DET == EMdgm_h::MAX_EXTRA_DET, "Format definition missmatch");
static_assert(EMdgm_f::MAX_EXTRA_DET_CLASSES == EMdgm_h::MAX_EXTRA_DET_CLASSES
              && EMdgm_i::MAX_EXTRA_DET_CLASSES == EMdgm_h::MAX_EXTRA_DET_CLASSES, "Format definition missmatch");
static_assert(EMdgm_f::MAX_NUM_TX_PULSES == EMdgm_h::MAX_NUM_TX_PULSES
              && EMdgm_i::MAX_NUM_TX_PULSES == EMdgm_h::MAX_NUM_TX_PULSES, "Format definition missmatch");
static_assert(EMdgm_f::MAX_DGM_SIZE == EMdgm_h::MAX_DGM_SIZE
              && EMdgm_i::MAX_DGM_SIZE == EMdgm_h::MAX_DGM_SIZE, "Format definition missmatch");
// Structures expected to be the same between revisions, let's check at least the size..
static_assert(sizeof(EMdgm_f::EMdgmHeader) == sizeof(EMdgm_h::EMdgmHeader)
              && sizeof(EMdgm_i::EMdgmHeader) == sizeof(EMdgm_h::EMdgmHeader), "Format definition missmatch");
static_assert(sizeof(EMdgm_f::EMdgmMpartition) == sizeof(EMdgm_h::EMdgmMpartition)
              && sizeof(EMdgm_i::EMdgmMpartition) == sizeof(EMdgm_h::EMdgmMpartition), "Format definition missmatch");


template <typename Struct>
bool read_struct(uint8_t *ptr, int &count, const int max_length, Struct &s)
{
  s = *(reinterpret_cast<Struct *>(ptr + count));
  count += sizeof(s);
  auto error = (count > max_length);
  if (error)
  {
    ROS_ERROR_STREAM("Exceeded max length when reading " << typeid(Struct).name());
  }
  return error;
}

template <typename body_S>
bool read_body_struct(uint8_t *ptr, int &count, const int max_length, body_S &s)
{
  s = *(reinterpret_cast<body_S *>(ptr + count));
  count += s.numBytesCmnPart;
  ROS_ERROR_STREAM_COND(s.numBytesCmnPart != sizeof(body_S), "body size diff");
  auto error = (count > max_length);
  if (error)
  {
    ROS_ERROR_STREAM("*After Common Part* In read_mrz, count=" << count << " exceeded max_length=" << max_length);
  }
  return error;
}

template <typename pingInfo_S>
bool read_ping_info_struct(uint8_t *ptr, int &count, const int max_length, pingInfo_S &s)
{
  s = *(reinterpret_cast<pingInfo_S *>(ptr + count));
  count += s.numBytesInfoData;
  ROS_ERROR_STREAM_COND(s.numBytesInfoData != sizeof(pingInfo_S), "pingInfo size diff");
  auto error = (count > max_length);
  if (error)
  {
    ROS_ERROR_STREAM("*After Ping Info* In read_mrz, count=" << count << " exceeded max_length=" << max_length);
  }
  return error;
}

template <typename txSectorInfo_S>
bool read_tx_sector_struct(uint8_t *ptr, int &count, const int max_length, txSectorInfo_S &s,
                           const size_t numBytesPerTxSector)
{
  s = *(reinterpret_cast<txSectorInfo_S *>(ptr + count));
  count += numBytesPerTxSector;
  ROS_ERROR_STREAM_COND(numBytesPerTxSector != sizeof(txSectorInfo_S), "tx Sector size diff");
  auto error = (count > max_length);
  if (error)
  {
    ROS_ERROR_STREAM("*After tx Sector Info* In read_mrz, count=" << count << " exceeded max_length=" << max_length);
  }
  return error;
}

template <typename rxInfo_S>
bool read_rx_info_struct(uint8_t *ptr, int &count, const int max_length, rxInfo_S &s)
{
  s = *(reinterpret_cast<rxInfo_S *>(ptr + count));
  count += s.numBytesRxInfo;
  ROS_ERROR_STREAM_COND(s.numBytesRxInfo != sizeof(rxInfo_S), "rx Sector size diff");
  auto error = (count > max_length);
  if (error)
  {
    ROS_ERROR_STREAM("*After rx Sector Info* In read_mrz, count=" << count << " exceeded max_length=" << max_length);
  }
  return error;
}

template <typename extraDet_S>
bool read_extra_det_struct(uint8_t *ptr, int &count, const int max_length, extraDet_S &s, size_t numBytesPerClass)
{
  s = *(reinterpret_cast<extraDet_S *>(ptr + count));
  count += numBytesPerClass;
  ROS_ERROR_STREAM_COND(numBytesPerClass != sizeof(extraDet_S), "extraDet size diff");
  auto error = (count > max_length);
  if (error)
  {
    ROS_ERROR_STREAM("*After extra Det Info* In read_mrz, count=" << count << " exceeded max_length=" << max_length);
  }
  return error;
}

/**
 * Reads the MRZ datagram based on the structure definition given as template parameter.
 * The caller must check the dgmVersion in the header structure of the KMALL message
 * to ensure that the correct EMdgmMRZ structure definition is used.
 *
 * If this is not the case, the function will fail.
 */
template <typename EMdgmMRZ_S>
static std::pair<bool, EMdgmMRZ_S> read_mrz(uint8_t *ptr, int max_length)
{
  EMdgmMRZ_S mrz;
  memset(&mrz, 0, sizeof(mrz));
  int count = 0;
  auto error = std::pair<bool, EMdgmMRZ_S>({false, {}});

  if (read_struct(ptr, count, max_length, mrz.header))
  {
    return error;
  }

  if (read_struct(ptr, count, max_length, mrz.partition))
  {
    return error;
  }

  if (read_body_struct(ptr, count, max_length, mrz.cmnPart))
  {
    return error;
  }

  if (read_ping_info_struct(ptr, count, max_length, mrz.pingInfo))
  {
    return error;
  }

  if (mrz.pingInfo.numTxSectors > EMdgm_f::MAX_NUM_TX_PULSES)
  {
    ROS_ERROR_STREAM("Read " << mrz.pingInfo.numTxSectors << " sectors vs maximum of " << EMdgm_f::MAX_NUM_TX_PULSES);
    return error;
  }

  for (int i = 0; i < mrz.pingInfo.numTxSectors; i++)
  {
    if (read_tx_sector_struct(ptr, count, max_length, mrz.sectorInfo[i], mrz.pingInfo.numBytesPerTxSector))
    {
      ROS_ERROR_STREAM("Failed decoding sector " << i << "/" << mrz.pingInfo.numTxSectors << " from ping count "
                                                 << mrz.cmnPart.pingCnt);
      return error;
    }
  }

  if (read_rx_info_struct(ptr, count, max_length, mrz.rxInfo))
  {
    return error;
  }

  if (mrz.rxInfo.numExtraDetectionClasses > EMdgm_f::MAX_EXTRA_DET_CLASSES)
  {
    ROS_ERROR_STREAM("Read " << mrz.rxInfo.numExtraDetectionClasses << " sectors vs maximum of "
                             << EMdgm_f::MAX_EXTRA_DET_CLASSES);
    return error;
  }

  for (int i = 0; i < mrz.rxInfo.numExtraDetectionClasses; i++)
  {
    if (read_extra_det_struct(ptr, count, max_length, mrz.extraDetClassInfo[i], mrz.rxInfo.numBytesPerClass))
    {
      return error;
    }
  }

  auto nb_soundings = mrz.rxInfo.numSoundingsMaxMain + mrz.rxInfo.numExtraDetections;
  if (nb_soundings > (EMdgm_f::MAX_NUM_BEAMS + EMdgm_f::MAX_EXTRA_DET))
  {
    ROS_ERROR_STREAM("Read " << nb_soundings << " sectors vs maximum of " << EMdgm_f::MAX_EXTRA_DET_CLASSES);
  }

  int SIsamples = 0;
  for (int i = 0; i < nb_soundings; i++)
  {
    if (read_struct(ptr, count, max_length, mrz.sounding[i]))
    {
      return error;
    }
    SIsamples += mrz.sounding[i].SInumSamples;
    ROS_ERROR_COND(sizeof(mrz.sounding[i]) != mrz.rxInfo.numBytesPerSounding, "Soundings size difference !");
  }

  for (int i = 0; i < SIsamples; i++)
  {
    if (read_struct(ptr, count, max_length, mrz.SIsample_desidB[i]))
    {
      return error;
    }
  }
  auto check_length = *(reinterpret_cast<uint32_t *>(ptr + count));
  count += sizeof(uint32_t);
  if(count != max_length){
    ROS_ERROR_STREAM_THROTTLE(10, "Count: " << count << "!= max_length: " << max_length);
  }

  if(mrz.header.numBytesDgm != check_length){
    ROS_ERROR_STREAM_THROTTLE(10,
                        "Header len: " << mrz.header.numBytesDgm << " Check len: " << check_length);  
  }
  
  return {true, mrz};
}

}  // namespace kmall
