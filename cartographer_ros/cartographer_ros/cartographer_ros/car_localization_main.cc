/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string>
#include <vector>
#include <iostream>
#include <memory>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer_ros/car_localization_node.h"
#include "cartographer_ros/map_reader.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace cartographer_ros {
namespace {

constexpr int kInfiniteSubscriberQueueSize = 0;

void LocRun() {
  auto file_resolver = cartographer::common::make_unique<
      cartographer::common::ConfigurationFileResolver>(
      std::vector<string>{FLAGS_configuration_directory});
  const string code =
      file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);

  std::cout << code << std::endl;

  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  const auto options = CreateNodeOptions(&lua_parameter_dictionary);
  constexpr double kTfBufferCacheTimeInSeconds = 1e6;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);

  std::shared_ptr<cartographer::mapping_2d::ProbabilityGrid> probability_grid;

  std::string stem("test");
  ReadPgmAndYamlToProbabilityGrid(probability_grid, stem);
  
  LocNode node(options, &tf_buffer, *probability_grid);
  node.Initialize();

  int trajectory_id = -1;
  std::unordered_set<string> expected_sensor_ids;

  // For 2D SLAM, subscribe to exactly one horizontal laser.
  ::ros::Subscriber laser_scan_subscriber;
  if (options.use_laser_scan) {
    laser_scan_subscriber = node.node_handle()->subscribe(
        kLaserScanTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)>(
            [&](const sensor_msgs::LaserScan::ConstPtr& msg) {
              node.map_builder_bridge()
                  ->sensor_bridge(trajectory_id)
                  ->HandleLaserScanMessage(kLaserScanTopic, msg);
            }));
    expected_sensor_ids.insert(kLaserScanTopic);
  }

  ::ros::Subscriber initial_pose_subscriber = node.node_handle()->subscribe("initialpose", 2, boost::function<void(const geometry_msgs::PoseWithCovarianceStampedConstPtr&)>(
            [&](const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
              node.map_builder_bridge()
                  ->SetPose(msg);
            }));

  trajectory_id = node.map_builder_bridge()->AddTrajectory(
      expected_sensor_ids, options.tracking_frame);

  ::ros::spin();

  node.map_builder_bridge()->FinishTrajectory(trajectory_id);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::LocRun();
  ::ros::shutdown();
}
