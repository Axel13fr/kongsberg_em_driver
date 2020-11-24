/**
* Copyright 2018 Woods Hole Oceanographic Institution
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
//
// Created by jvaccaro on 5/28/19.
//
#include "ds_kongsberg/kongsberg_em2040.h"

#include <list>
#include <gtest/gtest.h>
#include <ros/param.h>

// Test fixture
class Em2040Test : public::testing::Test
{
 public:
  std::string pass_path = ros::param::param<std::string>("pass_path", "kmall_pass.BIN");
  std::string fail_path = ros::param::param<std::string>("fail_path", "kmall_fail.BIN");
//  ds_kongsberg::KongsbergEM2040* node = new ds_kongsberg::KongsbergEM2040();
  static void SetUpTestCase()
  {
    ros::Time::init();
  }
};

TEST_F(Em2040Test, alwaysPass)
{
  EXPECT_TRUE(true);
}

TEST_F(Em2040Test, parsePass)
{
//  auto node = new ds_kongsberg::KongsbergEM2040();
  ds_kongsberg::KongsbergEM2040 nh;
  long long int index = 0;
  const auto test_ls = std::list<std::pair<bool, int>>{
    {true, 1104}, // #IIP
    {true, 1308}, // #IOP
    {true, 1192}, // #SVP
    {true, 19572},// #SKM
    {true, 74},   // #SCL
    {true, 156},  // #SPO
    {true, 156},  // #CPO
    {true, 74},   // #SCL
    {true, 156},  // #SPO
    {true, 156},  // #CPO
    {true, 55304},// #MRZ
    {true, 55236},// #MRZ
    {true, 19572},// #SKM
    {true, 74},   // #SCL
    {true, 156},  // #SPO
    {true, 156},  // #CPO
    {true, 74},   // #SCL
    {true, 156},  // #SPO
    {true, 156},  // #CPO
    {true, 19572},// #SKM
    {true, 55398},// #MRZ
    {true, 55336},// #MRZ
    {true, 19572},// #MRZ
    {true, 74},   // #SCL
    {true, 156},  // #SPO
    {true, 156},  // #CPO
    {true, 74},   // #SCL
    {true, 156},  // #SPO
    {true, 156},  // #CPO
    {true, 56332},// #MRZ
    {true, 55880},// #MRZ
    {true, 19572},// #SKM
    {true, 74},   // #SCL
    {true, 156},  // #SPO
    {true, 156}  // #CPO
  };
  for (auto test_l : test_ls){
    auto pass = test_l.first;
    auto size = test_l.second;
    ROS_ERROR_STREAM(" >> Reading " << size << " bytes from index " << index);
    auto byte_msg = ds_core_msgs::RawData{};
    bool ok = false;
    byte_msg.data.resize(size);
    uint8_t foo[size];
    FILE* file = fopen(pass_path.c_str(), "r");
    if (file!=nullptr)
    {
      fseek(file, index, SEEK_SET);
      fread(&foo, 1, size, file);
      fclose(file);
      for (int i=0; i<size; i++)
      {
        byte_msg.data[i] = foo[i];
      }
      byte_msg.ds_header.io_time = ros::Time::now();
      ok = nh.parse_data(byte_msg);
    }
    EXPECT_EQ(pass, ok);
    index += size;
  }

}

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{


  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_em2040_parse");
  auto ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}