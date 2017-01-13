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

#include "cartographer_ros/car_localization_bridge.h"

#include "cartographer_ros/assets_writer.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/occupancy_grid.h"
#include "cartographer_ros_msgs/TrajectorySubmapList.h"

namespace cartographer_ros {

CarLocalizationBridge::CarLocalizationBridge(const NodeOptions& options,
                                   tf2_ros::Buffer* const tf_buffer,
                                   const cartographer::mapping_2d::ProbabilityGrid& probability_grid)
    : probability_grid_(probability_grid), options_(options),
      map_builder_(options.map_builder_options, probability_grid_),
      tf_buffer_(tf_buffer) {
      }

int CarLocalizationBridge::AddTrajectory(
    const std::unordered_set<string>& expected_sensor_ids,
    const string& tracking_frame) {
  const int trajectory_id =
      map_builder_.AddTrajectoryBuilder(expected_sensor_ids);
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

  CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
  sensor_bridges_[trajectory_id] =
      cartographer::common::make_unique<SensorBridge>(
          tracking_frame, options_.lookup_transform_timeout_sec, tf_buffer_,
          map_builder_.GetTrajectoryBuilder(trajectory_id));
  return trajectory_id;
}

void CarLocalizationBridge::FinishTrajectory(const int trajectory_id) {
  LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

  CHECK_EQ(sensor_bridges_.count(trajectory_id), 1);
  map_builder_.FinishTrajectory(trajectory_id);
  sensor_bridges_.erase(trajectory_id);
}

std::unique_ptr<nav_msgs::OccupancyGrid>
CarLocalizationBridge::BuildOccupancyGrid() {
	namespace carto = ::cartographer;
  
  CHECK(options_.map_builder_options.use_trajectory_builder_2d())
      << "Publishing OccupancyGrids for 3D data is not yet supported";
   
  std::unique_ptr<nav_msgs::OccupancyGrid> occupancy_grid = cartographer::common::make_unique<nav_msgs::OccupancyGrid>();
  occupancy_grid->header.stamp = ::ros::Time::now();
  occupancy_grid->header.frame_id = options_.map_frame;
  occupancy_grid->info.map_load_time = occupancy_grid->header.stamp;

  carto::mapping_2d::CellLimits cell_limits = probability_grid_.limits().cell_limits();
  //probability_grid_.ComputeCroppedLimits(&offset, &cell_limits);
  const double resolution = probability_grid_.limits().resolution();
  Eigen::Array2i offset(0, 0);

  occupancy_grid->info.resolution = resolution;
  occupancy_grid->info.width = cell_limits.num_y_cells;
  occupancy_grid->info.height = cell_limits.num_x_cells;

  occupancy_grid->info.origin.position.x =
      probability_grid_.limits().max().x() -
      (offset.y() + cell_limits.num_y_cells) * resolution;
  occupancy_grid->info.origin.position.y =
      probability_grid_.limits().max().y() -
      (offset.x() + cell_limits.num_x_cells) * resolution;
  occupancy_grid->info.origin.position.z = 0.;
  occupancy_grid->info.origin.orientation.w = 1.;
  occupancy_grid->info.origin.orientation.x = 0.;
  occupancy_grid->info.origin.orientation.y = 0.;
  occupancy_grid->info.origin.orientation.z = 0.;
std::cout << probability_grid_.limits().max().x() << std::endl;
std::cout << probability_grid_.limits().max().y() << std::endl;
std::cout << occupancy_grid->info.origin.position.x << std::endl;
std::cout << occupancy_grid->info.origin.position.y << std::endl;
  occupancy_grid->data.resize(cell_limits.num_x_cells * cell_limits.num_y_cells,
                              -1);
  for (const Eigen::Array2i& xy_index :
       carto::mapping_2d::XYIndexRangeIterator(cell_limits)) {
    if (probability_grid_.IsKnown(xy_index + offset)) {
      const int value = carto::common::RoundToInt(
          (probability_grid_.GetProbability(xy_index + offset) -
           carto::mapping::kMinProbability) *
          100. /
          (carto::mapping::kMaxProbability - carto::mapping::kMinProbability));
      CHECK_LE(0, value);
      CHECK_GE(100, value);
      occupancy_grid->data[(cell_limits.num_x_cells - xy_index.x()) *
                               cell_limits.num_y_cells -
                           xy_index.y() - 1] = value;
    }
  }
	   
  return occupancy_grid;
}

std::unordered_map<int, CarLocalizationBridge::TrajectoryState>
CarLocalizationBridge::GetTrajectoryStates() {
  std::unordered_map<int, TrajectoryState> trajectory_states;
  for (const auto& entry : sensor_bridges_) {
    const int trajectory_id = entry.first;
    const SensorBridge& sensor_bridge = *entry.second;

    const cartographer::mapping::TrajectoryBuilder* const trajectory_builder =
        map_builder_.GetTrajectoryBuilder(trajectory_id);
    const cartographer::mapping::TrajectoryBuilder::PoseEstimate pose_estimate =
        trajectory_builder->pose_estimate();
    if (cartographer::common::ToUniversal(pose_estimate.time) < 0) {
      continue;
    }

    trajectory_states[trajectory_id] = {
        pose_estimate,
        cartographer::transform::Rigid3d::Identity(),
        sensor_bridge.tf_bridge().LookupToTracking(pose_estimate.time,
                                                   options_.published_frame)};
  }
  return trajectory_states;
}

SensorBridge* CarLocalizationBridge::sensor_bridge(const int trajectory_id) {
  return sensor_bridges_.at(trajectory_id).get();
}

}  // namespace cartographer_ros
