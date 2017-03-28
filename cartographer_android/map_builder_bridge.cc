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

#include "cartographer_android/map_builder_bridge.h"

#include "cartographer_android/assets_writer.h"

namespace cartographer_android {

MapBuilderBridge::MapBuilderBridge(const NodeOptions& options)
    : options_(options),
      map_builder_(options.map_builder_options, &constant_data_){}

int MapBuilderBridge::AddTrajectory(
    const std::unordered_set<string>& expected_sensor_ids,
    const string& tracking_frame) {
  const int trajectory_id =
      map_builder_.AddTrajectoryBuilder(expected_sensor_ids);
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

  CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
  sensor_bridges_[trajectory_id] =
      cartographer::common::make_unique<SensorBridge>(
          tracking_frame, options_.lookup_transform_timeout_sec,
          map_builder_.GetTrajectoryBuilder(trajectory_id));
  return trajectory_id;
}

void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
  LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

  CHECK_EQ(sensor_bridges_.count(trajectory_id), 1);
  map_builder_.FinishTrajectory(trajectory_id);
  map_builder_.sparse_pose_graph()->RunFinalOptimization();
  sensor_bridges_.erase(trajectory_id);
}

void MapBuilderBridge::WriteAssets(const string& stem) {
  const auto trajectory_nodes =
      map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
  if (trajectory_nodes.empty()) {
    LOG(WARNING) << "No data was collected and no assets will be written.";
  } else {
    LOG(INFO) << "Writing assets with stem '" << stem << "'...";
    cartographer_android::WriteAssets(trajectory_nodes, options_, stem);
  }
}

std::vector<cartographer::mapping::TrajectoryNode> MapBuilderBridge::GetTrajectoryNodes() {
	return map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
}

SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id) {
  return sensor_bridges_.at(trajectory_id).get();
}

}  // namespace cartographer_android
