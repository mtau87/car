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

#ifndef CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H_
#define CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H_

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "cartographer/mapping/map_builder.h"
#include "cartographer_android/node_options.h"
#include "cartographer_android/sensor_bridge.h"

namespace cartographer_android {

class MapBuilderBridge {
 public:
  struct TrajectoryState {
    cartographer::mapping::TrajectoryBuilder::PoseEstimate pose_estimate;
    cartographer::transform::Rigid3d local_to_map;
    std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;
  };

  MapBuilderBridge(const NodeOptions& options);

  MapBuilderBridge(const MapBuilderBridge&) = delete;
  MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;

  int AddTrajectory(const std::unordered_set<string>& expected_sensor_ids,
                    const string& tracking_frame);
  void FinishTrajectory(int trajectory_id);
  void WriteAssets(const string& stem);

  std::vector<cartographer::mapping::TrajectoryNode> GetTrajectoryNodes();

  SensorBridge* sensor_bridge(int trajectory_id);

 private:
  const NodeOptions options_;

  std::deque<cartographer::mapping::TrajectoryNode::ConstantData>
      constant_data_;
  cartographer::mapping::MapBuilder map_builder_;
  std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;
};

}  // namespace cartographer_android

#endif  // CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H_
