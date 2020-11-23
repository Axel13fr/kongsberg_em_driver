//
// Created by jvaccaro on 7/25/19.
//

#include "../ds_kongsberg/kongsberg_em2040_util.h"

#include <vector>
#include <gtest/gtest.h>
#include <ros/param.h>

class Em2040Util : public::testing::Test
{
 public:
  std::string xml_filename = "/home/jvaccaro/code/sentry_ws/src/ds_kongsberg/ds_kongsberg/src/test/test_xml_load.xml";
  static void SetUpTestCase()
  {
  }
};

TEST_F(Em2040Util, test_string_split_out_xml_params_pass){
//  std::string xml = "<TOKEN>\n<ID>par1</ID>\n<VALUE>val1</VALUE>\n</TOKEN>\n<TOKEN>\n<ID>par2</ID>\n<VALUE>val2</VALUE>\n</TOKEN>\n<TOKEN>\n<ID>par3</ID>\n<VALUE>val3</VALUE>\n</TOKEN>\n<TOKEN>\n<ID>par4</ID>\n<XXVALUE>val4</XXVALUE>\n</TOKEN>\n<TOKEN>\n<ID>par5</ID>\n<VALUE>val5</VALUE>\n</TOKEN>";
  std::string xml = "<TOKEN>\n  <ID>UDP6N</ID>\n<HIDE>0</HIDE>\n  <TYPE>select</TYPE>\n  <VALUE>Main net</VALUE>\n </TOKEN>\n <TOKEN>\n  <ID>UDP6P</ID>\n  <HIDE>0</HIDE>\n  <TYPE>number</TYPE>\n  <VALUE>0</VALUE>\n </TOKEN>\n <TOKEN>\n  <ID>RX1S</ID>\n  <HIDE>1</HIDE>\n </TOKEN>\n <TOKEN>\n  <ID>RX2S</ID>\n  <HIDE>1</HIDE>\n </TOKEN>\n</ALL_TOKENS>";
  std::vector<std::string> expected_params = {"UDP6N", "UDP6P"};
  std::vector<std::string> expected_vals = {"Main net", "0"};
  std::vector<std::string> params, vals;
  std::tie(params, vals) = ds_kongsberg::string_split_out_xml_params(xml);
  ASSERT_EQ(params.size(), vals.size());
  ASSERT_EQ(params.size(), expected_params.size());
  for (int i=0; i<params.size(); i++){
    ASSERT_STREQ(params[i].data(), expected_params[i].data());
    ASSERT_STREQ(vals[i].data(), expected_vals[i].data());
  }
}

TEST_F(Em2040Util, test_file_split_out_xml_params_pass){
  std::vector<std::string> expected_params = {"p1", "p2", "p3", "p5"};
  std::vector<std::string> expected_vals = {"v1", "v2", "v3", "v5"};
  std::vector<std::string> params, vals;
  std::tie(params, vals) = ds_kongsberg::file_split_out_xml_params(xml_filename);
  ASSERT_EQ(params.size(), vals.size());
  ASSERT_EQ(params.size(), expected_params.size());
  for (int i=0; i<params.size(); i++){
    ASSERT_STREQ(params[i].data(), expected_params[i].data());
    ASSERT_STREQ(vals[i].data(), expected_vals[i].data());
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
//  ros::init(argc, argv, "test_em2040_utils");
  auto ret = RUN_ALL_TESTS();
//  ros::shutdown();
  return ret;
}