//
// Created by jvaccaro on 7/17/19.
//

#ifndef PROJECT_KONGSBERG_EM2040_UTIL_H
#define PROJECT_KONGSBERG_EM2040_UTIL_H

#define M_PI_RAD 3.14159
#define M_PI_DEG 180.0

#include "boost/date_time/posix_time/posix_time_io.hpp"
#include "boost/date_time/posix_time/posix_time_types.hpp"
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

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

} //namespace

#endif //PROJECT_KONGSBERG_EM2040_UTIL_H
