// Replay actual full-run traces against the firmware's continuous-motion guard.
#include "MotionWatchdog.h"
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

void require(bool condition, const char* message) {
  if (!condition) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}

bool replay(const char* path) {
  std::ifstream input(path);
  require(bool(input), "open captured trace");
  bool reading = false, initialized = false;
  MotionWatchdog guard;
  for (std::string line; std::getline(input, line);) {
    if (line.find("trace_us,") == 0) { reading = true; continue; }
    if (line.find("TRACE_END") == 0) break;
    if (!reading) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    uint32_t us = 0, loop = 0, a = 0, b = 0;
    float angle = 0;
    require(bool(row >> us >> loop >> a >> b >> angle), "parse trace row");
    if (!initialized) { guard.reset(angle, 0); initialized = true; }
    if (guard.stalled(angle, us)) {
      std::cout << "STALL at " << us << "us in " << path << '\n';
      return true;
    }
  }
  require(initialized, "trace was present");
  std::cout << "NO STALL in " << path << '\n';
  return false;
}

int main(int argc, char** argv) {
  require(argc == 3, "provide known-stalled trace and best moving trace");
  require(replay(argv[1]), "detect recorded mid-run stop with I=0");
  require(!replay(argv[2]), "preserve best recorded three-second trial");
  MotionWatchdog guard;
  const uint32_t origin = UINT32_MAX - 50000;
  guard.reset(0, origin);
  require(!guard.stalled(0, origin + 1199999u), "allow startup grace through clock wrap");
  require(guard.stalled(0, origin + 1200000u), "detect persistent stop through clock wrap");
  guard.reset(0, 0);
  for (uint32_t now = 100000; now <= 3000000; now += 100000)
    require(!guard.stalled(float(now) * 1e-6f, now), "allow continuous one-rad/s movement");
  require(!guard.stalled(3, 3500000), "allow short pause");
  require(guard.stalled(3, 3600000), "stop after 600ms without progress");
  std::cout << "PASS: recorded stop, recorded moving trial, grace, clock wrap, ongoing motion, pause\n";
}
