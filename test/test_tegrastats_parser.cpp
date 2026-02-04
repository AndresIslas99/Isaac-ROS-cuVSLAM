#include <gtest/gtest.h>
#include "agv_slam/tegrastats_parser.hpp"

using agv_slam::parse_tegrastats_line;
using agv_slam::TegraStats;

// Real Orin AGX tegrastats output sample
static const char * ORIN_SAMPLE =
  "RAM 5678/62795MB (lfb 12345x4MB) SWAP 0/31398MB (cached 0MB) "
  "CPU [12%@2201,34%@2201,5%@2201,8%@2201,22%@2201,15%@2201,0%@2201,3%@2201,"
  "45%@2201,10%@2201,7%@2201,18%@2201] "
  "EMC_FREQ 0% GR3D_FREQ 78% GR3D2_FREQ 0% "
  "cv0@42.5C cpu@48.5C gpu@45C tj@52C soc2@43C soc0@44.5C "
  "VDD_GPU_SOC 3456mW/3456mW VDD_CPU_CV 2345mW/2345mW";

TEST(TegraStatsParser, GpuPercentParsed)
{
  auto stats = parse_tegrastats_line(ORIN_SAMPLE);
  EXPECT_TRUE(stats.valid);
  EXPECT_DOUBLE_EQ(stats.gpu_percent, 78.0);
}

TEST(TegraStatsParser, CpuPercentAveraged)
{
  auto stats = parse_tegrastats_line(ORIN_SAMPLE);
  EXPECT_TRUE(stats.valid);
  // 12+34+5+8+22+15+0+3+45+10+7+18 = 179, / 12 cores ≈ 14.9
  EXPECT_NEAR(stats.cpu_percent, 179.0 / 12.0, 0.1);
}

TEST(TegraStatsParser, TemperatureMaxExtracted)
{
  auto stats = parse_tegrastats_line(ORIN_SAMPLE);
  EXPECT_TRUE(stats.valid);
  // Highest temp in the sample is tj@52C
  EXPECT_DOUBLE_EQ(stats.thermal_zone, 52.0);
}

TEST(TegraStatsParser, PowerWattsSummed)
{
  auto stats = parse_tegrastats_line(ORIN_SAMPLE);
  EXPECT_TRUE(stats.valid);
  // VDD_GPU_SOC 3456mW + VDD_CPU_CV 2345mW = 5801mW = 5.801W
  EXPECT_NEAR(stats.power_watts, 5.801, 0.001);
}

TEST(TegraStatsParser, RamParsed)
{
  auto stats = parse_tegrastats_line(ORIN_SAMPLE);
  EXPECT_TRUE(stats.valid);
  EXPECT_EQ(stats.ram_used_mb, 5678u);
  EXPECT_EQ(stats.ram_total_mb, 62795u);
}

TEST(TegraStatsParser, EmptyAndMalformedInput)
{
  {
    auto stats = parse_tegrastats_line("");
    EXPECT_FALSE(stats.valid);
    EXPECT_DOUBLE_EQ(stats.gpu_percent, 0.0);
  }
  {
    auto stats = parse_tegrastats_line("not a tegrastats line at all");
    EXPECT_TRUE(stats.valid);  // valid=true but fields default to 0
    EXPECT_DOUBLE_EQ(stats.gpu_percent, 0.0);
    EXPECT_DOUBLE_EQ(stats.cpu_percent, 0.0);
    EXPECT_EQ(stats.ram_used_mb, 0u);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
