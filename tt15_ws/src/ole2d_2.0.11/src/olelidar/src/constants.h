#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace olelidar {

static constexpr auto kNaNF = std::numeric_limits<float>::quiet_NaN();
static constexpr auto kNaND = std::numeric_limits<double>::quiet_NaN();
static constexpr float kTau = M_PI * 2;
static constexpr float deg2rad(float deg) { return deg * M_PI / 180.0; }
static constexpr float rad2deg(float rad) { return rad * 180.0 / M_PI; }

// Raw olei packet constants and structures.
// 1 packet = 150 blocks
// 1 block = 1 sequence
// 1 sequence = 1 firing = 1 point
// 1 point = 8 bytes
static constexpr int kPointBytes = 8;
static constexpr int kPointsPerBlock = 1;

// According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
// valid packets with readings up to 130.0.
static constexpr float kDistanceMax = 20.0;         // [m]
static constexpr float kDistanceResolution = 0.001;  // [m]  2d lidar,

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;  //rsv
static const uint16_t LOWER_BANK = 0xddff;  //rsv

/** Special Defines for olei support **/
static constexpr double kSingleFiringNs = 2304;  // [ns] rsv,1s/10hz/1600points = 62500
static constexpr double kFiringCycleNs = 55296;  // [ns] rsv,                   = 62500
static constexpr double kSingleFiringRatio = kSingleFiringNs / kFiringCycleNs;  // 1

// The information from two firing sequences of 16 lasers is contained in each
// data block. Each packet contains the data from 24 firing sequences in 12 data
// blocks.
static constexpr int kFiringsPerSequence = 1;
static constexpr int kSequencesPerBlock = 1;
static constexpr int kBlocksPerPacket = 150;
static constexpr int kSequencesPerPacket =
    kSequencesPerBlock * kBlocksPerPacket;  // 150

inline int LaserId2Row(int id) {
  const auto index = (id % 2 == 0) ? id / 2 : id / 2 + kFiringsPerSequence / 2;
  return kFiringsPerSequence - index - 1;
}

static constexpr uint16_t kMaxRawAzimuth = 35999;
static constexpr float kAzimuthResolution = 0.01f;

// static constexpr float kMinElevation = deg2rad(-15.0f);
// static constexpr float kMaxElevation = deg2rad(15.0f);
// static constexpr float kDeltaElevation =
//    (kMaxElevation - kMinElevation) / (kFiringsPerSequence - 1);

inline constexpr float Raw2Azimuth(uint16_t raw) {
  // According to the user manual,
  return deg2rad(static_cast<float>(raw) * kAzimuthResolution);
}

/// p55 9.3.1.2
inline constexpr float Raw2Distance(uint16_t raw) {
  return static_cast<float>(raw) * kDistanceResolution;
}

/// p51 8.3.1
inline constexpr float AzimuthResolutionDegree(int rpm) {
  // rpm % 60 == 0
  return rpm / 60.0 * 360.0 * kFiringCycleNs / 1e9;
}

}  // namespace olelidar
