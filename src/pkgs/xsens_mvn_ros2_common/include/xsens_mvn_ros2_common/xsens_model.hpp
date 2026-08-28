// Copyright (c) 2026, Xsens Technologies B.V.
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

// Maps XME SDK segment names (PascalCase, or SDK-specific labels like "Sternum")
// to the canonical snake_case names used as TF frame suffixes by both nodes.
inline std::string xmeSegmentToCanonical(const std::string & sdk_name)
{
  static const std::unordered_map<std::string, std::string> mapping = {
    {"Pelvis", "pelvis"},
    {"L5", "l5"},
    {"L3", "l3"},
    {"T12", "t12"},
    {"T8", "t8"},
    {"Sternum", "t8"},  // Some XME SDK versions label the T8 tracker segment "Sternum"
    {"Neck", "neck"},
    {"Head", "head"},
    {"RightShoulder", "right_shoulder"},
    {"RightUpperArm", "right_upper_arm"},
    {"RightForeArm", "right_forearm"},
    {"RightHand", "right_hand"},
    {"LeftShoulder", "left_shoulder"},
    {"LeftUpperArm", "left_upper_arm"},
    {"LeftForeArm", "left_forearm"},
    {"LeftHand", "left_hand"},
    {"RightUpperLeg", "right_upper_leg"},
    {"RightLowerLeg", "right_lower_leg"},
    {"RightFoot", "right_foot"},
    {"RightToe", "right_toe"},
    {"LeftUpperLeg", "left_upper_leg"},
    {"LeftLowerLeg", "left_lower_leg"},
    {"LeftFoot", "left_foot"},
    {"LeftToe", "left_toe"},
  };
  const auto it = mapping.find(sdk_name);
  return (it != mapping.end()) ? it->second : sdk_name;
}

// Finger-tracking segment naming (MANUS gloves via MVN)
//
// When finger tracking is enabled we get extra segments in addition
// to the standard 23 body segments. We have not tested with props.
//
// A hand's block is 20 segments: 5 fingers x 4 phalanges, no leading carpus
// segment (confirmed against a real MANUS glove recording)

inline std::string fingerSegmentName(const std::string & side, int position)
/// \param side "left" or "right"
/// \param position 0-based index of the segment within this hand's block, 
// in the order received (ascending segment ID)
{
  const int finger = position / 4 + 1;    // 1 (thumb) .. 5 (pinky), per MVN order
  const int phalange = position % 4 + 1;  // 1 (metacarpal) .. 4 (distal)
  std::ostringstream oss;
  oss << side << "_hand_f" << finger << "_p" << phalange;
  return oss.str();
}

/// Parses a name produced by fingerSegmentName() back into (side, finger,
/// phalange). Returns false if `seg` doesn't match the finger-segment naming
/// pattern.
inline bool parseFingerSegmentName(
  const std::string & seg, std::string & side, int & finger, int & phalange)
{
  for (const char * s : {"left", "right"}) {
    const std::string prefix = std::string(s) + "_hand_";
    if (seg.rfind(prefix, 0) != 0) {
      continue;
    }
    side = s;
    const std::string suffix = seg.substr(prefix.size());
    const auto sep = suffix.find("_p");
    if (suffix.size() < 4 || suffix[0] != 'f' || sep == std::string::npos) {
      return false;
    }
    try {
      finger = std::stoi(suffix.substr(1, sep - 1));
      phalange = std::stoi(suffix.substr(sep + 2));
    } catch (const std::exception &) {
      return false;
    }
    return true;
  }
  return false;
}

/// Returns the canonical snake_case parent segment name for a given canonical segment name.
/// Returns an empty string for the root segment (pelvis), which is broadcast directly to world.
inline std::string kineticParent(const std::string & seg)
{
  static const std::unordered_map<std::string, std::string> kParent = {
    {"l5", "pelvis"}, {"l3", "l5"}, {"t12", "l3"}, {"t8", "t12"},
    {"neck", "t8"}, {"head", "neck"},
    {"left_shoulder", "t8"}, {"left_upper_arm", "left_shoulder"},
    {"left_forearm", "left_upper_arm"}, {"left_hand", "left_forearm"},
    {"right_shoulder", "t8"}, {"right_upper_arm", "right_shoulder"},
    {"right_forearm", "right_upper_arm"}, {"right_hand", "right_forearm"},
    {"left_upper_leg", "pelvis"}, {"left_lower_leg", "left_upper_leg"},
    {"left_foot", "left_lower_leg"}, {"left_toe", "left_foot"},
    {"right_upper_leg", "pelvis"}, {"right_lower_leg", "right_upper_leg"},
    {"right_foot", "right_lower_leg"}, {"right_toe", "right_foot"},
  };
  const auto it = kParent.find(seg);
  if (it != kParent.end()) {
    return it->second;
  }

  // Dynamic fallback for finger-tracking segments: every phalange parents to
  // the previous phalange on the same finger, and each finger's first
  // phalange parents directly to the hand
  std::string side;
  int finger = 0, phalange = 0;
  if (parseFingerSegmentName(seg, side, finger, phalange)) {
    if (phalange <= 1) {
      return side + "_hand";
    }
    std::ostringstream oss;
    oss << side << "_hand_f" << finger << "_p" << (phalange - 1);
    return oss.str();
  }

  return std::string();
}

struct XsensModelNames
{
  const std::vector<std::string> links = {
    "base_link",
    "pelvis",
    "l5",
    "l3",
    "t12",
    "t8",
    "neck",
    "head",
    "right_shoulder",
    "right_upper_arm",
    "right_forearm",
    "right_hand",
    "left_shoulder",
    "left_upper_arm",
    "left_forearm",
    "left_hand",
    "right_upper_leg",
    "right_lower_leg",
    "right_foot",
    "right_toe",
    "left_upper_leg",
    "left_lower_leg",
    "left_foot",
    "left_toe",
    "generic_link"};

  const std::vector<std::string> joints = {
    "l5_s1",
    "l4_l3",
    "l1_t12",
    "t9_t8",
    "t1_c7",
    "c1_head",
    "right_c7_shoulder",
    "right_shoulder",
    "right_elbow",
    "right_wrist",
    "left_c7_shoulder",
    "left_shoulder",
    "left_elbow",
    "left_wrist",
    "right_hip",
    "right_knee",
    "right_ankle",
    "right_ballfoot",
    "left_hip",
    "left_knee",
    "left_ankle",
    "left_ballfoot",
    "t8_head_NA",
    "t8_left_upper_arm_NA",
    "t8_right_upper_arm_NA",
    "pelvis_t8_NA",
    "pelvis_pelvis_NA",
    "pelvis_t8_v2_NA"};
};
