//
// Created by jvaccaro on 7/25/19.
//

#include "../../src/em_driver_library/EM_driver/kongsberg_em2040_util.h"

#include <vector>
#include <fstream>
#include <gtest/gtest.h>
#include <ros/param.h>

class Em2040Util : public ::testing::Test
{
 public:
  std::string xml_filename = "test/EM2040_40_cfg.xml";
  static void SetUpTestCase()
  {
  }
};

TEST_F(Em2040Util, test_string_split_out_xml_params_pass)
{
  //  std::string xml =
  //  "<TOKEN>\n<ID>par1</ID>\n<VALUE>val1</VALUE>\n</TOKEN>\n<TOKEN>\n<ID>par2</ID>\n<VALUE>val2</VALUE>\n</TOKEN>\n<TOKEN>\n<ID>par3</ID>\n<VALUE>val3</VALUE>\n</TOKEN>\n<TOKEN>\n<ID>par4</ID>\n<XXVALUE>val4</XXVALUE>\n</TOKEN>\n<TOKEN>\n<ID>par5</ID>\n<VALUE>val5</VALUE>\n</TOKEN>";
  std::string xml =
      "<TOKEN>\n  <ID>UDP6N</ID>\n<HIDE>0</HIDE>\n  <TYPE>select</TYPE>\n  <VALUE>Main net</VALUE>\n </TOKEN>\n "
      "<TOKEN>\n  <ID>UDP6P</ID>\n  <HIDE>0</HIDE>\n  <TYPE>number</TYPE>\n  <VALUE>0</VALUE>\n </TOKEN>\n <TOKEN>\n  "
      "<ID>RX1S</ID>\n  <HIDE>1</HIDE>\n </TOKEN>\n <TOKEN>\n  <ID>RX2S</ID>\n  <HIDE>1</HIDE>\n "
      "</TOKEN>\n</ALL_TOKENS>";
  std::vector<std::string> expected_params = {"UDP6N", "UDP6P"};
  std::vector<std::string> expected_vals = {"Main net", "0"};
  std::vector<std::string> params, vals;
  std::tie(params, vals) = kongsberg_em::string_split_out_xml_params(xml);
  ASSERT_EQ(params.size(), vals.size());
  ASSERT_EQ(params.size(), expected_params.size());
  for (int i = 0; i < params.size(); i++)
  {
    ASSERT_STREQ(params[i].data(), expected_params[i].data());
    ASSERT_STREQ(vals[i].data(), expected_vals[i].data());
  }
}

TEST_F(Em2040Util, test_file_split_out_xml_params_pass)
{
  std::vector<std::string> expected_params = {"SDPM1", "SDPT1","SDMI","SDMA","SSPA1","SSSA1","SSPC","SSSC"};
  std::vector<std::string> expected_vals = {"300kHz", "Auto","2","200","55","30","101","71"};
  std::vector<std::string> params, vals;

  {
    std::ifstream in(xml_filename, std::ios::in | std::ios::binary);
    if (not in.is_open())
    {
      std::cout << "Failed opening " << xml_filename << std::endl;
    }
    ASSERT_TRUE(in.is_open());
    in.close();
  }
  std::tie(params, vals) = kongsberg_em::file_split_out_xml_params(xml_filename);
  ASSERT_EQ(params.size(), vals.size());
  ASSERT_EQ(params.size(), 312);

  for (int i = 0; i < expected_params.size(); i++)
  {
    auto found = false;
    for (int j = 0; j < params.size(); j++)
    {
      if (params[j] == expected_params[i])
      {
        found = true;
        ASSERT_EQ(expected_vals[i], vals[j]);
      }
    }
    ASSERT_TRUE(found);
//    auto elm = std::find(params.begin(), params.end(), expected_params[i]);
//    ASSERT_STREQ(params[i].data(), expected_params[i].data());
//    ASSERT_STREQ(vals[i].data(), expected_vals[i].data());
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  //  ros::init(argc, argv, "test_em2040_utils");
  auto ret = RUN_ALL_TESTS();
  //  ros::shutdown();
  return ret;
}
