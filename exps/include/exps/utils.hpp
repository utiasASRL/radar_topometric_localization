#pragma once

#include <filesystem>
#include <mutex>
#include <random>

#include "rclcpp/rclcpp.hpp"

#include "vtr_logging/logging.hpp"

namespace fs = std::filesystem;

namespace vtr {
namespace exps {

std::string random_string(std::size_t length) {
  const std::string CHARACTERS = "abcdefghijklmnopqrstuvwxyz";
  std::random_device random_device;
  std::mt19937 generator(random_device());
  std::uniform_int_distribution<> distribution(0, CHARACTERS.size() - 1);
  std::string result;
  for (std::size_t i = 0; i < length; ++i) result += CHARACTERS[distribution(generator)];
  return result;
}

int64_t getStampFromPath(const std::string &path) {
  std::vector<std::string> parts;
  boost::split(parts, path, boost::is_any_of("/"));
  std::string stem = parts[parts.size() - 1];
  boost::split(parts, stem, boost::is_any_of("."));
  int64_t time1 = std::stoll(parts[0]);
  return time1 * 1000;
}

float getFloatFromByteArray(char *byteArray, uint index) { return *((float *)(byteArray + index)); }

Eigen::MatrixXd load_lidar(const std::string &path) {
  std::ifstream ifs(path, std::ios::binary);
  std::vector<char> buffer(std::istreambuf_iterator<char>(ifs), {});
  uint float_offset = 4;
  uint fields = 6;  // x, y, z, i, r, t
  uint point_step = float_offset * fields;
  uint N = floor(buffer.size() / point_step);
  Eigen::MatrixXd pc(Eigen::MatrixXd::Ones(N, fields));
  for (uint i = 0; i < N; ++i) {
    uint bufpos = i * point_step;
    for (uint j = 0; j < fields; ++j) {
      pc(i, j) = getFloatFromByteArray(buffer.data(), bufpos + j * float_offset);
    }
  }
  return pc;
}

tactic::EdgeTransform load_T_robot_lidar(const fs::path &path) {
#if true
  std::ifstream ifs(path / "calib" / "T_applanix_lidar.txt", std::ios::in);

  Eigen::Matrix4d T_applanix_lidar_mat;
  for (size_t row = 0; row < 4; row++)
    for (size_t col = 0; col < 4; col++) ifs >> T_applanix_lidar_mat(row, col);

  Eigen::Matrix4d yfwd2xfwd;
  yfwd2xfwd << 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  tactic::EdgeTransform T_robot_lidar(Eigen::Matrix4d(yfwd2xfwd * T_applanix_lidar_mat),
                                      Eigen::Matrix<double, 6, 6>::Zero());
#else
  Eigen::Matrix4d T_robot_lidar_mat;
  // clang-format off
  /// results from HERO paper
  T_robot_lidar_mat <<  0.68297386,  0.73044281,  0.        ,  0.26      ,
                       -0.73044281,  0.68297386,  0.        ,  0.        ,
                        0.        ,  0.        ,  1.        , -0.21      ,
                        0.        ,  0.        ,  0.        ,  1.        ;
  // clang-format on
  tactic::EdgeTransform T_robot_lidar(T_robot_lidar_mat, Eigen::Matrix<double, 6, 6>::Zero());
#endif

  return T_robot_lidar;
}

tactic::EdgeTransform load_T_robot_radar(const fs::path &path) {
#if true
  std::ifstream ifs1(path / "calib" / "T_applanix_lidar.txt", std::ios::in);
  std::ifstream ifs2(path / "calib" / "T_radar_lidar.txt", std::ios::in);

  Eigen::Matrix4d T_applanix_lidar_mat;
  for (size_t row = 0; row < 4; row++)
    for (size_t col = 0; col < 4; col++) ifs1 >> T_applanix_lidar_mat(row, col);

  Eigen::Matrix4d T_radar_lidar_mat;
  for (size_t row = 0; row < 4; row++)
    for (size_t col = 0; col < 4; col++) ifs2 >> T_radar_lidar_mat(row, col);

  Eigen::Matrix4d yfwd2xfwd;
  yfwd2xfwd << 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  tactic::EdgeTransform T_robot_radar(Eigen::Matrix4d(yfwd2xfwd * T_applanix_lidar_mat * T_radar_lidar_mat.inverse()),
                                      Eigen::Matrix<double, 6, 6>::Zero());
#else
  (void)path;
  // robot frame == radar frame
  tactic::EdgeTransform T_robot_radar(Eigen::Matrix4d(Eigen::Matrix4d::Identity()),
                                      Eigen::Matrix<double, 6, 6>::Zero());
#endif

  return T_robot_radar;
}

Eigen::Matrix3d toRoll(const double &r) {
  Eigen::Matrix3d roll;
  roll << 1, 0, 0, 0, cos(r), sin(r), 0, -sin(r), cos(r);
  return roll;
}

Eigen::Matrix3d toPitch(const double &p) {
  Eigen::Matrix3d pitch;
  pitch << cos(p), 0, -sin(p), 0, 1, 0, sin(p), 0, cos(p);
  return pitch;
}

Eigen::Matrix3d toYaw(const double &y) {
  Eigen::Matrix3d yaw;
  yaw << cos(y), sin(y), 0, -sin(y), cos(y), 0, 0, 0, 1;
  return yaw;
}

Eigen::Matrix3d rpy2rot(const double &r, const double &p, const double &y) { return toRoll(r) * toPitch(p) * toYaw(y); }

tactic::EdgeTransform load_T_enu_lidar_init(const fs::path &path) {
  std::ifstream ifs(path / "applanix" / "lidar_poses.csv", std::ios::in);

  std::string header;
  std::getline(ifs, header);

  std::string first_pose;
  std::getline(ifs, first_pose);

  std::stringstream ss{first_pose};
  std::vector<double> gt;
  for (std::string str; std::getline(ss, str, ',');) gt.push_back(std::stod(str));

  Eigen::Matrix4d T_mat = Eigen::Matrix4d::Identity();
  T_mat.block<3, 3>(0, 0) = rpy2rot(gt[7], gt[8], gt[9]);
  T_mat.block<3, 1>(0, 3) << gt[1], gt[2], gt[3];

  tactic::EdgeTransform T(T_mat);
  T.setZeroCovariance();

  return T;
}

tactic::EdgeTransform load_T_enu_radar_init(const fs::path &path) {
  std::ifstream ifs(path / "applanix" / "radar_poses.csv", std::ios::in);

  std::string header;
  std::getline(ifs, header);

  std::string first_pose;
  std::getline(ifs, first_pose);

  std::stringstream ss{first_pose};
  std::vector<double> gt;
  for (std::string str; std::getline(ss, str, ',');) gt.push_back(std::stod(str));

  Eigen::Matrix4d T_mat = Eigen::Matrix4d::Identity();
  T_mat.block<3, 3>(0, 0) = rpy2rot(gt[7], gt[8], gt[9]);
  T_mat.block<3, 1>(0, 3) << gt[1], gt[2], gt[3];

  tactic::EdgeTransform T(T_mat);
  T.setZeroCovariance();

  return T;
}

struct TestControl {
  TestControl(const rclcpp::Node::SharedPtr &node) {
    // parameters to control the playback
    // clang-format off
    play_ = node->declare_parameter<bool>("control_test.play", play_);
    terminate_ = node->declare_parameter<bool>("control_test.terminate", terminate_);
    // clang-format on
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    rcl_interfaces::msg::IntegerRange delay_range;
    delay_range.from_value = 0;
    delay_range.to_value = 1000;
    delay_range.step = 1;
    param_desc.integer_range.emplace_back(delay_range);
    delay_ = node->declare_parameter<int>("control_test.delay_millisec", delay_, param_desc);
    parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        node->get_node_base_interface(), node->get_node_topics_interface(), node->get_node_graph_interface(),
        node->get_node_services_interface());
    parameter_event_sub_ =
        parameter_client_->on_parameter_event([&](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
          std::lock_guard<std::mutex> lock(mutex_);
          for (auto &changed_parameter : event->changed_parameters) {
            const auto &type = changed_parameter.value.type;
            const auto &name = changed_parameter.name;
            const auto &value = changed_parameter.value;
            CLOG(WARNING, "test") << "Received parameter change event with name: " << name;

            if (type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
              if (name == "control_test.delay_millisec") {
                delay_ = value.integer_value;
              }
            } else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
              if (name == "control_test.play") {
                play_ = value.bool_value;
              } else if (name == "control_test.terminate") {
                terminate_ = value.bool_value;
              }
            }
          }
        });
  }

  bool play() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return play_;
  }

  bool terminate() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return terminate_;
  }

  int delay() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return delay_;
  }

 private:
  mutable std::mutex mutex_;

  bool play_ = true;
  bool terminate_ = false;
  int delay_ = 0;  // milliseconds

  rclcpp::AsyncParametersClient::SharedPtr parameter_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
};

}  // namespace exps
}  // namespace vtr
