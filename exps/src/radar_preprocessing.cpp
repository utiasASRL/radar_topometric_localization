#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/string.hpp"

#include "vtr_common/timing/utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_radar/pipeline.hpp"
#include "vtr_tactic/pipelines/factory.hpp"
#include "vtr_tactic/rviz_tactic_callback.hpp"
#include "vtr_tactic/tactic.hpp"

#include "exps/utils.hpp"

namespace fs = std::filesystem;
using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::tactic;
using namespace vtr::exps;

int64_t getStampFromPath(const std::string &path) {
  std::vector<std::string> parts;
  boost::split(parts, path, boost::is_any_of("/"));
  std::string stem = parts[parts.size() - 1];
  boost::split(parts, stem, boost::is_any_of("."));
  int64_t time1 = std::stoll(parts[0]);
  return time1 * 1000;
}

EdgeTransform load_T_robot_radar(const fs::path &path) {
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

  EdgeTransform T_robot_radar(Eigen::Matrix4d(yfwd2xfwd * T_applanix_lidar_mat * T_radar_lidar_mat.inverse()),
                              Eigen::Matrix<double, 6, 6>::Zero());
#else
  (void)path;
  // robot frame == radar frame
  EdgeTransform T_robot_radar(Eigen::Matrix4d(Eigen::Matrix4d::Identity()), Eigen::Matrix<double, 6, 6>::Zero());
#endif

  return T_robot_radar;
}

int main(int argc, char **argv) {
  // disable eigen multi-threading
  Eigen::setNbThreads(1);

  rclcpp::init(argc, argv);
  const std::string node_name = "radar_preprocessing_" + random_string(10);
  auto node = rclcpp::Node::make_shared(node_name);

  // odometry sequence directory
  const auto odo_dir_str = node->declare_parameter<std::string>("odo_dir", "/tmp");
  fs::path odo_dir{utils::expand_user(utils::expand_env(odo_dir_str))};

  // Output directory
  const auto data_dir_str = node->declare_parameter<std::string>("data_dir", "/tmp");
  fs::path data_dir{utils::expand_user(utils::expand_env(data_dir_str))};

  // Configure logging
  const auto log_to_file = node->declare_parameter<bool>("log_to_file", false);
  const auto log_debug = node->declare_parameter<bool>("log_debug", false);
  const auto log_enabled = node->declare_parameter<std::vector<std::string>>("log_enabled", std::vector<std::string>{});
  std::string log_filename;
  if (log_to_file) {
    // Log into a subfolder of the data directory (if requested to log)
    auto log_name = "vtr-" + timing::toIsoFilename(timing::clock::now());
    log_filename = data_dir / (log_name + ".log");
  }
  configureLogging(log_filename, log_debug, log_enabled);

  CLOG(WARNING, "test") << "Odometry Directory: " << odo_dir.string();
  CLOG(WARNING, "test") << "Output Directory: " << data_dir.string();

  std::vector<std::string> parts;
  boost::split(parts, odo_dir_str, boost::is_any_of("/"));
  auto stem = parts.back();
  boost::replace_all(stem, "-", "_");
  CLOG(WARNING, "test") << "Publishing status to topic: " << (stem + "_radar_preprocessing");
  const auto status_publisher = node->create_publisher<std_msgs::msg::String>(stem + "_radar_preprocessing", 1);

  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / "graph").string(), false);

  // Pipeline
  auto pipeline_factory = std::make_shared<ROSPipelineFactory>(node);
  auto pipeline = pipeline_factory->get("pipeline");
  auto pipeline_output = pipeline->createOutputCache();
  // some modules require node for visualization
  pipeline_output->node = node;

  // Tactic Callback
  auto callback = std::make_shared<RvizTacticCallback>(node);

  // Tactic
  auto tactic = std::make_shared<Tactic>(Tactic::Config::fromROS(node), pipeline, pipeline_output, graph, callback);
  tactic->setPipeline(PipelineMode::Idle);

  // Frame and transforms
  std::string robot_frame = "robot";
  std::string radar_frame = "radar";

  const auto T_robot_radar = load_T_robot_radar(odo_dir);
  const auto T_radar_robot = T_robot_radar.inverse();
  CLOG(WARNING, "test") << "Transform from " << robot_frame << " to " << radar_frame << " has been set to"
                        << T_radar_robot;

  auto tf_sbc = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  auto msg = tf2::eigenToTransform(Eigen::Affine3d(T_radar_robot.inverse().matrix()));
  msg.header.frame_id = robot_frame;
  msg.child_frame_id = radar_frame;
  tf_sbc->sendTransform(msg);

  const auto clock_publisher = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  // List of radar data
  std::vector<fs::directory_entry> files;
  for (const auto &dir_entry : fs::directory_iterator{odo_dir / "radar"})
    if (!fs::is_directory(dir_entry)) files.push_back(dir_entry);
  std::sort(files.begin(), files.end());
  CLOG(WARNING, "test") << "Found " << files.size() << " radar data";

  // thread handling variables
  TestControl test_control(node);

  // main loop
  int frame = 0;
  auto it = files.begin();
  while (it != files.end()) {
    if (!rclcpp::ok()) break;
    rclcpp::spin_some(node);
    if (test_control.terminate()) break;
    if (!test_control.play()) continue;
    std::this_thread::sleep_for(std::chrono::milliseconds(test_control.delay()));

    ///
    const auto timestamp = getStampFromPath(it->path().string());
    const auto scan = cv::imread(it->path().string(), cv::IMREAD_GRAYSCALE);

    CLOG(WARNING, "test") << "Loading radar frame " << frame << " with timestamp " << timestamp;

    // publish clock for sim time
    auto time_msg = rosgraph_msgs::msg::Clock();
    time_msg.clock = rclcpp::Time(timestamp);
    clock_publisher->publish(time_msg);

    // Convert message to query_data format and store into query_data
    auto query_data = std::make_shared<radar::RadarQueryCache>();

    // some modules require node for visualization
    query_data->node = node;

    // set timestamp
    query_data->stamp.emplace(timestamp);

    // make up some environment info (not important)
    tactic::EnvInfo env_info;
    env_info.terrain_type = 0;
    query_data->env_info.emplace(env_info);

    // set radar frame
    query_data->scan.emplace(scan);

    // fill in the vehicle to sensor transform and frame name
    query_data->T_s_r.emplace(T_radar_robot);

    // execute the pipeline
    tactic->input(query_data);

    std_msgs::msg::String status_msg;
    status_msg.data =
        "Finished processing radar frame " + std::to_string(frame) + " with timestamp " + std::to_string(timestamp);
    status_publisher->publish(status_msg);

    ++it;
    ++frame;
  }

  rclcpp::shutdown();

  tactic.reset();
  callback.reset();
  pipeline.reset();
  pipeline_factory.reset();
  CLOG(WARNING, "test") << "Saving pose graph and reset.";
  graph->save();
  graph.reset();
  CLOG(WARNING, "test") << "Saving pose graph and reset. - DONE!";
}