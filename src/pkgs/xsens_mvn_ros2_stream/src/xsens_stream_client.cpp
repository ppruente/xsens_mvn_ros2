// Copyright (c) 2026, Xsens Technologies B.V.
// SPDX-License-Identifier: BSD-3-Clause
#include "xsens_mvn_ros2_stream/xsens_stream_client.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <system_error>

#include <xsens_mvn_ros2_common/xsens_model.hpp>

XsensStreamClient::XsensStreamClient(rclcpp::Logger logger, const int & udp_port)
: m_logger(logger), m_udpPort(udp_port)
{}

bool XsensStreamClient::init()
{
  // Initialize UDP communication
  m_udpSocket = std::make_shared<Socket>(IP_UDP);
  if (!m_udpSocket->bind(static_cast<uint16_t>(m_udpPort))) {
    RCLCPP_ERROR(
      m_logger,
      "Error binding Xsens UDP port %d. "
      "Check: (1) Is another process already using this port? "
      "(2) Is MVN Studio configured to stream on this port?",
      m_udpPort);
    return false;
  }

  m_dataAcquisitionThread = boost::thread(&XsensStreamClient::dataAcquisitionCallback, this);

  // Wait for Xsens data acquisition activation (with timeout)
  auto init_start = std::chrono::steady_clock::now();
  while (!m_clientActive) {
    if (std::chrono::steady_clock::now() - init_start > std::chrono::seconds(30)) {
      RCLCPP_ERROR(
        m_logger,
        "Timeout waiting for data acquisition after 30 seconds. "
        "Check: (1) Is MVN Studio streaming? "
        "(2) Is the correct UDP port (%d) configured in MVN Studio? "
        "(3) Is a firewall blocking UDP traffic?",
        m_udpPort);
      return false;
    }
    boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
  }
  RCLCPP_INFO(m_logger, "Xsens stream client initialized.");

  return true;
}

void XsensStreamClient::dataAcquisitionCallback()
{
  RCLCPP_INFO(m_logger, "Xsens stream client starting data acquisition.");

  if (!buildXsensModel()) {
    RCLCPP_ERROR(
      m_logger,
      "Failure building human model. Data acquisition stopped. "
      "Check: (1) Is MVN Studio streaming quaternion and joint angle datagrams? "
      "(2) Is the MVN datagram format compatible?");
    m_clientActive = false;
  } else {
    RCLCPP_INFO(m_logger, "Human model built successfully.");
    m_clientActive = true;
  }

  while (m_clientActive) {
    if (readData()) {
      std::lock_guard<std::mutex> lock(m_dataMutex);
      updateJointAngles();
      updateLinkPoses();
      updateLinkLinearTwists();
      updateLinkAngularTwists();
      updateCOM();
    }
  }
}

bool XsensStreamClient::readData()
{
  // Read data from UDP socket
  if (auto read_bytes = m_udpSocket->read(m_dataBuffer, MAX_MVN_DATAGRAM_SIZE); read_bytes > 0) {
    m_parserManager.readDatagram(m_dataBuffer);
    m_lastDataTimeNs = std::chrono::steady_clock::now().time_since_epoch().count();
    return true;
  }
  // Receive timeout (EAGAIN/EWOULDBLOCK) is not a real error — the stream may
  // have paused briefly (e.g. during calibration in MVN Studio).  Continue the
  // acquisition loop so data is picked up again once the stream resumes.
  if (errno == EAGAIN || errno == EWOULDBLOCK) {
    return false;
  }
  RCLCPP_ERROR(
    m_logger,
    "Error reading data from UDP socket: %s. Stopping acquisition. "
    "Check: (1) Is MVN Studio still streaming? "
    "(2) Has the network connection been interrupted?",
    strerror(errno));
  m_clientActive = false;
  return false;
}

bool XsensStreamClient::buildXsensModel()
{
  // Initialize human data handler
  m_humanData = std::make_shared<xsens_mvn_ros2::HumanDataHandler>(m_logger);
  m_linkNameList.clear();
  m_jointNameList.clear();

  XsensModelNames xsens_model_names;

  auto quaternion_datagram = waitForQuaternionDatagram();
  auto joint_angles_datagram = waitForJointAnglesDatagram();

  // Get links
  //
  // Segment IDs 1..23 are the standard body model, indexed
  // directly into xsens_model_names.links 
  // Any segments beyond that are assumed to be MANUS finger-tracking data, 
  // appended by MVN as [left hand block][right hand block]
  // Each hand block is 20 segments (5 fingers x 4 phalanges)
  constexpr int kBodySegmentCount = 23;

  RCLCPP_INFO(
    m_logger, "Building model: %zu links available.",
    quaternion_datagram.getData().size());

  const int total_segments = static_cast<int>(quaternion_datagram.getData().size());
  const int extra_segments = std::max(0, total_segments - kBodySegmentCount);
  const int segments_per_hand = extra_segments / 2;
  if (extra_segments % 2 != 0) {
    RCLCPP_WARN(
      m_logger,
      "Received %d segments beyond the standard %d-segment body model (odd number) "
      "so the left/right hand split below is likely wrong.",
      extra_segments, kBodySegmentCount);
  }
  if (segments_per_hand > 0) {
    RCLCPP_INFO(
      m_logger, "MANUS gloves finger tracking data detected: %d segments per hand.",
      segments_per_hand);
  }

  for (const auto & xsens_link : quaternion_datagram.getData()) {
    const int segment_id = xsens_link.segmentId;
    std::string link_name;

    if (segment_id >= 1 && segment_id <= kBodySegmentCount) {
      link_name = xsens_model_names.links[segment_id];
    } else if (
      segments_per_hand > 0 && segment_id > kBodySegmentCount &&
      segment_id <= kBodySegmentCount + 2 * segments_per_hand)
    {
      const int offset = segment_id - kBodySegmentCount - 1;  // 0-based, both hands
      const bool is_left = offset < segments_per_hand;
      const int position = is_left ? offset : offset - segments_per_hand;
      link_name = fingerSegmentName(is_left ? "left" : "right", position);
    } else {
      RCLCPP_WARN(
        m_logger,
        "Segment ID %d is outside the known body/finger model range "
        "(%d total segments this frame); skipping.",
        segment_id, total_segments);
      continue;
    }

    if (!m_humanData->setLink(link_name, xsens_mvn_ros2::Link(link_name))) {
      RCLCPP_ERROR(m_logger, "Error inserting link '%s'. Aborting model build.",
        link_name.c_str());
      return false;
    } else {
      m_linkNameList.push_back(link_name);
    }
  }
  if (m_humanData->getLinks().empty()) {
    RCLCPP_ERROR(m_logger, "No link elements found in quaternion datagram.");
    return false;
  }

  RCLCPP_INFO(
    m_logger, "Building model: %zu joints available.",
    joint_angles_datagram.getData().size());
  for (size_t joint_cnt = 0; joint_cnt < joint_angles_datagram.getData().size(); joint_cnt++) {
    auto xsens_joint = joint_angles_datagram.getData()[joint_cnt];
    RCLCPP_DEBUG(m_logger, "  Joint %zu: %s -> %s (%s)",
      joint_cnt,
      m_linkNameList[xsens_joint.parentSegmentId - 1].c_str(),
      m_linkNameList[xsens_joint.childSegmentId - 1].c_str(),
      xsens_model_names.joints[joint_cnt].c_str());
    if (!m_humanData->setJoint(
          xsens_model_names.joints[joint_cnt],
          xsens_mvn_ros2::Joint(xsens_model_names.joints[joint_cnt])))
    {
      RCLCPP_ERROR(m_logger, "Error inserting joint '%s'. Aborting model build.",
        xsens_model_names.joints[joint_cnt].c_str());
      return false;
    } else {
      m_jointNameList.push_back(xsens_model_names.joints[joint_cnt]);
    }
  }
  if (m_humanData->getJoints().empty()) {
    RCLCPP_ERROR(m_logger, "No joint elements found in joint angles datagram.");
    return false;
  }

  return true;
}

QuaternionDatagram XsensStreamClient::waitForQuaternionDatagram()
{
  RCLCPP_INFO(m_logger, "Waiting for quaternion datagram...");
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (!m_parserManager.getQuaternionDatagram()) {
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_ERROR(
        m_logger,
        "Timeout waiting for quaternion datagram after 30 seconds. "
        "Check: (1) Is MVN Studio streaming? "
        "(2) Is the quaternion datagram enabled in MVN Studio network settings? "
        "(3) Is the correct UDP port (%d) configured?",
        m_udpPort);
      return QuaternionDatagram();
    }
    if (!readData()) {
      return QuaternionDatagram();
    }
  }
  RCLCPP_INFO(m_logger, "Quaternion datagram received.");
  return *(m_parserManager.getQuaternionDatagram());
}

JointAnglesDatagram XsensStreamClient::waitForJointAnglesDatagram()
{
  RCLCPP_INFO(m_logger, "Waiting for joint angles datagram...");
  auto deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (!m_parserManager.getJointAnglesDatagram()) {
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_ERROR(
        m_logger,
        "Timeout waiting for joint angles datagram after 30 s. "
        "Check: (1) Is MVN Studio streaming? "
        "(2) Is the joint angles datagram enabled in "
        "MVN Studio network settings? "
        "(3) Is the correct UDP port (%d) configured?",
        m_udpPort);
      return JointAnglesDatagram();
    }
    if (!readData()) {
      return JointAnglesDatagram();
    }
  }
  RCLCPP_INFO(m_logger, "Joint angles datagram received.");
  return *(m_parserManager.getJointAnglesDatagram());
}

void XsensStreamClient::updateJointAngles()
{
  auto joint_angles = m_parserManager.getJointAnglesDatagram();
  if (joint_angles) {
    // Update joint angles according to N-pose
    m_humanData->setJointAngles(
      "l5_s1", jointAngleToEigenVector3d(joint_angles->getItem(1, 2), 1, 1, 1));
    m_humanData->setJointAngles(
      "l4_l3", jointAngleToEigenVector3d(joint_angles->getItem(2, 3), 1, 1, 1));
    m_humanData->setJointAngles(
      "l1_t12", jointAngleToEigenVector3d(joint_angles->getItem(3, 4), 1, 1, 1));
    m_humanData->setJointAngles(
      "t9_t8", jointAngleToEigenVector3d(joint_angles->getItem(4, 5), 1, 1, 1));
    m_humanData->setJointAngles(
      "t1_c7", jointAngleToEigenVector3d(joint_angles->getItem(5, 6), 1, 1, 1));
    m_humanData->setJointAngles(
      "c1_head", jointAngleToEigenVector3d(joint_angles->getItem(6, 7), 1, 1, 1));
    m_humanData->setJointAngles(
      "right_c7_shoulder", jointAngleToEigenVector3d(joint_angles->getItem(5, 8), -1, 1, -1));
    m_humanData->setJointAngles(
      "right_shoulder", jointAngleToEigenVector3d(joint_angles->getItem(8, 9), -1, 1, -1));
    m_humanData->setJointAngles(
      "right_elbow", jointAngleToEigenVector3d(joint_angles->getItem(9, 10), -1, 1, -1));
    m_humanData->setJointAngles(
      "right_wrist", jointAngleToEigenVector3d(joint_angles->getItem(10, 11), -1, 1, -1));
    m_humanData->setJointAngles(
      "left_c7_shoulder", jointAngleToEigenVector3d(joint_angles->getItem(5, 12), 1, -1, -1));
    m_humanData->setJointAngles(
      "left_shoulder", jointAngleToEigenVector3d(joint_angles->getItem(12, 13), 1, -1, -1));
    m_humanData->setJointAngles(
      "left_elbow", jointAngleToEigenVector3d(joint_angles->getItem(13, 14), 1, -1, -1));
    m_humanData->setJointAngles(
      "left_wrist", jointAngleToEigenVector3d(joint_angles->getItem(14, 15), 1, -1, -1));
    m_humanData->setJointAngles(
      "right_hip", jointAngleToEigenVector3d(joint_angles->getItem(1, 16), -1, 1, -1));
    m_humanData->setJointAngles(
      "right_knee", jointAngleToEigenVector3d(joint_angles->getItem(16, 17), -1, 1, 1));
    m_humanData->setJointAngles(
      "right_ankle", jointAngleToEigenVector3d(joint_angles->getItem(17, 18), -1, 1, -1));
    m_humanData->setJointAngles(
      "right_ballfoot", jointAngleToEigenVector3d(joint_angles->getItem(18, 19), -1, 1, -1));
    m_humanData->setJointAngles(
      "left_hip", jointAngleToEigenVector3d(joint_angles->getItem(1, 20), 1, -1, -1));
    m_humanData->setJointAngles(
      "left_knee", jointAngleToEigenVector3d(joint_angles->getItem(20, 21), 1, -1, 1));
    m_humanData->setJointAngles(
      "left_ankle", jointAngleToEigenVector3d(joint_angles->getItem(21, 22), 1, -1, -1));
    m_humanData->setJointAngles(
      "left_ballfoot", jointAngleToEigenVector3d(joint_angles->getItem(22, 23), 1, -1, -1));

    // Add +90° to elbows angle
    xsens_mvn_ros2::Joint elbow_joint;
    m_humanData->getJoint("right_elbow", elbow_joint);
    elbow_joint.state.angles[2] -= 90.0;
    m_humanData->setJoint("right_elbow", elbow_joint);
    m_humanData->getJoint("left_elbow", elbow_joint);
    elbow_joint.state.angles[2] += 90.0;
    m_humanData->setJoint("left_elbow", elbow_joint);
  }
}

void XsensStreamClient::updateLinkPoses()
{
  auto quaternion_datagram = m_parserManager.getQuaternionDatagram();
  if (!quaternion_datagram) {
    return;
  }
  for (size_t link_cnt = 0; link_cnt < m_linkNameList.size(); link_cnt++) {
    auto link_pose = quaternion_datagram->getItem(static_cast<int32_t>(link_cnt) + 1);
    Eigen::Vector3d link_pos;
    link_pos << link_pose.sensorPos[0], link_pose.sensorPos[1], link_pose.sensorPos[2];
    Eigen::Quaterniond link_orient(
      link_pose.quatRotation[0], link_pose.quatRotation[1], link_pose.quatRotation[2],
      link_pose.quatRotation[3]);
    m_humanData->setLinkPose(m_linkNameList[link_cnt], link_pos, link_orient);
  }

  Eigen::Quaterniond quat_x_rot_90neg(0.7071068, -0.7071068, 0.0, 0.0);
  rotateLink("right_upper_arm", quat_x_rot_90neg);
  rotateLink("right_forearm", quat_x_rot_90neg);
  rotateLink("right_hand", quat_x_rot_90neg);

  Eigen::Quaterniond quat_x_rot_90pos(0.7071068, 0.7071068, 0.0, 0.0);
  rotateLink("left_upper_arm", quat_x_rot_90pos);
  rotateLink("left_forearm", quat_x_rot_90pos);
  rotateLink("left_hand", quat_x_rot_90pos);
}

void XsensStreamClient::rotateLink(
  const std::string & link_name,
  const Eigen::Quaterniond & quat) const
{
  xsens_mvn_ros2::Link link_to_rotate;
  if (!m_humanData->getLink(link_name, link_to_rotate)) {
    RCLCPP_WARN_ONCE(m_logger, "Link '%s' not found for rotation correction.", link_name.c_str());
    return;
  }
  link_to_rotate.state.orientation = link_to_rotate.state.orientation * quat;
  m_humanData->setLink(link_name, link_to_rotate);
}

void XsensStreamClient::updateLinkLinearTwists()
{
  auto linear_segment_kinematics_datagram = m_parserManager.getLinearSegmentKinematicsDatagram();
  if (!linear_segment_kinematics_datagram) {
    return;
  }
  for (size_t link_cnt = 0; link_cnt < m_linkNameList.size(); link_cnt++) {
    // Get linear kinematics from datagram structure
    auto link_linear_kinematics =
      linear_segment_kinematics_datagram->getItem(static_cast<int32_t>(link_cnt) + 1);

    // Get link
    xsens_mvn_ros2::Link link;
    if (!m_humanData->getLink(m_linkNameList[link_cnt], link)) {
      RCLCPP_WARN_ONCE(m_logger, "Link '%s' not found in human data handler (linear twists).",
        m_linkNameList[link_cnt].c_str());
    } else {
      link.state.velocity.linear << link_linear_kinematics.velocity[0],
        link_linear_kinematics.velocity[1], link_linear_kinematics.velocity[2];
      link.state.acceleration.linear << link_linear_kinematics.acceleration[0],
        link_linear_kinematics.acceleration[1], link_linear_kinematics.acceleration[2];

      m_humanData->setLinkState(m_linkNameList[link_cnt], link.state);
    }
  }
}

void XsensStreamClient::updateLinkAngularTwists()
{
  auto angular_segment_kinematics_datagram = m_parserManager.getAngularSegmentKinematicsDatagram();
  if (!angular_segment_kinematics_datagram) {
    return;
  }
  for (size_t link_cnt = 0; link_cnt < m_linkNameList.size(); link_cnt++) {
    // Get angular kinematics from datagram structure
    auto link_angular_kinematics =
      angular_segment_kinematics_datagram->getItem(static_cast<int32_t>(link_cnt) + 1);
    // Get link
    xsens_mvn_ros2::Link link;
    if (!m_humanData->getLink(m_linkNameList[link_cnt], link)) {
      RCLCPP_WARN_ONCE(m_logger, "Link '%s' not found in human data handler (angular twists).",
        m_linkNameList[link_cnt].c_str());
    } else {
      link.state.velocity.angular << link_angular_kinematics.angularVeloc[0] * M_PI / 180,
        link_angular_kinematics.angularVeloc[1] * M_PI / 180,
        link_angular_kinematics.angularVeloc[2] * M_PI / 180;
      link.state.acceleration.angular << link_angular_kinematics.angularAccel[0] * M_PI / 180,
        link_angular_kinematics.angularAccel[1] * M_PI / 180,
        link_angular_kinematics.angularAccel[2] * M_PI / 180;
      m_humanData->setLinkState(m_linkNameList[link_cnt], link.state);
    }
  }
}

void XsensStreamClient::updateCOM()
{
  auto com_datagram = m_parserManager.getCenterOfMassDatagram();
  if (!com_datagram) {
    return;
  }
  Eigen::Vector3d com;
  com << com_datagram->getData()[0], com_datagram->getData()[1], com_datagram->getData()[2];
  m_humanData->setCOM(com);
}

Eigen::Vector3d XsensStreamClient::jointAngleToEigenVector3d(
  const JointAngle & joint_angle, const double & x_axis, const double & y_axis,
  const double & z_axis) const
{
  // Convert JointAngle object to a Eigen Vector3d rotating axes to align in resting position N-Pose
  Eigen::Vector3d joint_angle_eigen_vec;
  joint_angle_eigen_vec[0] = x_axis * joint_angle.rotation[0];
  // TEMPORARY HORRIBLE SOLUTION
  joint_angle_eigen_vec[1] = z_axis * joint_angle.rotation[2];
  joint_angle_eigen_vec[2] = y_axis * joint_angle.rotation[1];
  return joint_angle_eigen_vec;
}

std::unordered_map<std::string,
  xsens_mvn_ros2::SegmentKinematics> XsensStreamClient::getSegments() const
{
  std::lock_guard<std::mutex> lock(m_dataMutex);
  std::unordered_map<std::string, xsens_mvn_ros2::SegmentKinematics> segments;
  if (!m_humanData) {
    return segments;
  }
  for (const auto & [name, link] : m_humanData->getLinks()) {
    xsens_mvn_ros2::SegmentKinematics kin;
    kin.position = link.state.position;
    kin.orientation = link.state.orientation;
    kin.velocity_linear = link.state.velocity.linear;
    kin.velocity_angular = link.state.velocity.angular;
    kin.accel_linear = link.state.acceleration.linear;
    kin.accel_angular = link.state.acceleration.angular;
    segments[name] = kin;
  }
  return segments;
}

std::vector<xsens_mvn_ros2::JointAngles> XsensStreamClient::getJoints() const
{
  std::lock_guard<std::mutex> lock(m_dataMutex);
  std::vector<xsens_mvn_ros2::JointAngles> joint_vec;
  if (!m_humanData) {
    return joint_vec;
  }
  for (const auto & [name, joint] : m_humanData->getJoints()) {
    joint_vec.push_back(
      {name, joint.state.angles[0] * M_PI / 180.0, joint.state.angles[1] * M_PI / 180.0,
        joint.state.angles[2] * M_PI / 180.0});
  }
  return joint_vec;
}

std::optional<Eigen::Vector3d> XsensStreamClient::getCOM() const
{
  std::lock_guard<std::mutex> lock(m_dataMutex);
  if (!m_humanData) {
    return std::nullopt;
  }
  return m_humanData->getCOM();
}

XsensStreamClient::~XsensStreamClient()
{
  m_clientActive = false;
  if (m_dataAcquisitionThread.joinable()) {
    m_dataAcquisitionThread.join();
  }
  RCLCPP_DEBUG(m_logger, "XsensStreamClient destroyed.");
}
