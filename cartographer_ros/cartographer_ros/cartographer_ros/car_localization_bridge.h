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

#ifndef CARTOGRAPHER_ROS_CAR_LOCALIZATION_BRIDGE_H_
#define CARTOGRAPHER_ROS_CAR_LOCALIZATION_BRIDGE_H_

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "cartographer/mapping_2d/car_localization_trajectory_builder.h"
#include "cartographer/mapping/car_localization_builder.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "nav_msgs/OccupancyGrid.h"

namespace cartographer_ros {

class CarLocalizationBridge {
 public:
  struct TrajectoryState {
    cartographer::mapping::TrajectoryBuilder::PoseEstimate pose_estimate;    
    cartographer::transform::Rigid3d local_to_map;
    std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;
  };

  CarLocalizationBridge(const NodeOptions& options, tf2_ros::Buffer* tf_buffer, const cartographer::mapping_2d::ProbabilityGrid& probability_grid);

  CarLocalizationBridge(const CarLocalizationBridge&) = delete;
  CarLocalizationBridge& operator=(const CarLocalizationBridge&) = delete;
  
  std::unique_ptr<nav_msgs::OccupancyGrid> BuildOccupancyGrid();

  int AddTrajectory(const std::unordered_set<string>& expected_sensor_ids,
                    const string& tracking_frame);

  void FinishTrajectory(int trajectory_id);

  std::unordered_map<int, TrajectoryState> GetTrajectoryStates();
  SensorBridge* sensor_bridge(int trajectory_id);

 private:
  const cartographer::mapping_2d::ProbabilityGrid& probability_grid_;
  const NodeOptions options_;
  cartographer::mapping::CarLocalizationBuilder map_builder_;
  tf2_ros::Buffer* const tf_buffer_;
  std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;  
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CAR_LOCALIZATION_BRIDGE_H_
