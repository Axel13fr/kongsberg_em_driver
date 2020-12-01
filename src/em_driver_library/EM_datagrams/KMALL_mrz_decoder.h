#pragma once

#include "EMdgmFormat.h"
#include "EMdgmFormat_h.h"
#include <utility>
#include <cstring>
#include <ros/console.h>

namespace kmall{

template <typename Struct>
bool read_struct(uint8_t *ptr, int &count, const int max_length, Struct &s)
{
  s = *(reinterpret_cast<Struct *>(ptr + count));
  count += sizeof(s);
  auto error = (count > max_length);
  if(error){
    ROS_ERROR_STREAM("Exceeded max length when reading " << typeid(Struct).name());
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

  if (read_struct(ptr, count, max_length, mrz.header))
  {
    return {false, {}};
  }

  if (read_struct(ptr, count, max_length, mrz.partition))
  {
    return {false, {}};
  }

  if (read_struct(ptr, count, max_length, mrz.cmnPart))
  {
    return {false, {}};
  }

  if (read_struct(ptr, count, max_length, mrz.pingInfo))
  {
    return {false, {}};
  }

  for (int i = 0; i < mrz.pingInfo.numTxSectors; i++)
  {
    if (read_struct(ptr, count, max_length, mrz.sectorInfo[i]))
    {
      return {false, {}};
    }
  }

  if (read_struct(ptr, count, max_length, mrz.rxInfo))
  {
    return {false, {}};
  }


  for (int i = 0; i < mrz.rxInfo.numExtraDetectionClasses; i++)
  {
    if (read_struct(ptr, count, max_length, mrz.extraDetClassInfo[i]))
    {
      return {false, {}};
    }
  }

  int SIsamples = 0;
  for (int i = 0; i < mrz.rxInfo.numSoundingsMaxMain + mrz.rxInfo.numExtraDetections; i++)
  {
    if (read_struct(ptr, count, max_length, mrz.sounding[i]))
    {
      return {false, {}};
    }
    SIsamples += mrz.sounding[i].SInumSamples;
  }

  for (int i = 0; i < SIsamples; i++)
  {
    if (read_struct(ptr, count, max_length, mrz.SIsample_desidB[i]))
    {
      return {false, {}};
    }
  }
  auto check_length = *(reinterpret_cast<uint32_t *>(ptr + count));
  count += sizeof(uint32_t);
  // ROS_ERROR_STREAM("Count: " << count << " max_length: "<<max_length);
  // ROS_ERROR_STREAM("Header len: " << mrz.header.numBytesDgm << " Check len: "<<check_length);
  // ROS_ERROR_STREAM("PING TIME: "<<mrz.header.time_sec);
  return {true, mrz};
}

}
