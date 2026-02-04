#pragma once

#include <string>
#include <regex>
#include <cstdint>

namespace agv_slam
{

struct TegraStats {
  double gpu_percent = 0.0;
  double cpu_percent = 0.0;
  double thermal_zone = 0.0;   // Max temperature in Celsius
  double power_watts = 0.0;    // Total board power
  uint64_t ram_used_mb = 0;
  uint64_t ram_total_mb = 0;
  bool valid = false;
};

/**
 * Parse a single line of tegrastats output from Jetson Orin.
 *
 * Example line:
 *   RAM 5678/62795MB (lfb 12345x4MB) SWAP 0/31398MB ... GR3D_FREQ 78%
 *   CPU [12%@1234,34%@2345,...] ... GPU 45%@1300 ... tj@52C ... VDD_GPU_SOC 3456mW/3456mW VDD_CPU_CV 2345mW/2345mW
 */
inline TegraStats parse_tegrastats_line(const std::string & line)
{
  TegraStats stats;

  if (line.empty()) {
    return stats;
  }

  // GPU usage: "GR3D_FREQ 78%" or "GPU 45%@1300"
  {
    std::regex re(R"((?:GR3D_FREQ|GPU)\s+(\d+)%)");
    std::smatch match;
    if (std::regex_search(line, match, re)) {
      stats.gpu_percent = std::stod(match[1].str());
    }
  }

  // CPU usage: "CPU [12%@1234,34%@2345,...]" — average all cores
  {
    std::regex re(R"((\d+)%@\d+)");
    auto begin = std::sregex_iterator(line.begin(), line.end(), re);
    auto end = std::sregex_iterator();
    double sum = 0.0;
    int count = 0;
    for (auto it = begin; it != end; ++it) {
      sum += std::stod((*it)[1].str());
      count++;
    }
    if (count > 0) {
      stats.cpu_percent = sum / count;
    }
  }

  // Temperature: "tj@52C" or "cpu@48.5C" — take the highest
  {
    std::regex re(R"((\w+)@([\d.]+)C)");
    auto begin = std::sregex_iterator(line.begin(), line.end(), re);
    auto end = std::sregex_iterator();
    double max_temp = 0.0;
    for (auto it = begin; it != end; ++it) {
      double temp = std::stod((*it)[2].str());
      if (temp > max_temp) {
        max_temp = temp;
      }
    }
    stats.thermal_zone = max_temp;
  }

  // Power: "VDD_GPU_SOC 3456mW/3456mW VDD_CPU_CV 2345mW/2345mW" — sum current values
  {
    std::regex re(R"((\d+)mW/\d+mW)");
    auto begin = std::sregex_iterator(line.begin(), line.end(), re);
    auto end = std::sregex_iterator();
    double total_mw = 0.0;
    for (auto it = begin; it != end; ++it) {
      total_mw += std::stod((*it)[1].str());
    }
    stats.power_watts = total_mw / 1000.0;
  }

  // RAM: "RAM 5678/62795MB"
  {
    std::regex re(R"(RAM\s+(\d+)/(\d+)MB)");
    std::smatch match;
    if (std::regex_search(line, match, re)) {
      stats.ram_used_mb = std::stoull(match[1].str());
      stats.ram_total_mb = std::stoull(match[2].str());
    }
  }

  stats.valid = true;
  return stats;
}

}  // namespace agv_slam
