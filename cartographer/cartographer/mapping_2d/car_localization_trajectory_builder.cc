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

#include "cartographer/mapping_2d/car_localization_trajectory_builder.h"
#include <iostream>
namespace cartographer {
namespace mapping_2d {

CarLocalizationTrajectoryBuilder::CarLocalizationTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options, const ProbabilityGrid& probability_grid)
    : options_(options),
      local_trajectory_builder_(options, probability_grid) {}

CarLocalizationTrajectoryBuilder::~CarLocalizationTrajectoryBuilder() {}

const Submaps* CarLocalizationTrajectoryBuilder::submaps() const {
  return NULL;
}

void CarLocalizationTrajectoryBuilder::AddRangefinderData(
    const common::Time time, const Eigen::Vector3f& origin,
    const sensor::PointCloud& ranges) {
    local_trajectory_builder_.AddHorizontalLaserFan(
        time, sensor::LaserFan{origin, ranges, {}, {}});
}

void CarLocalizationTrajectoryBuilder::AddImuData(
    const common::Time time, const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity) {
  local_trajectory_builder_.AddImuData(time, linear_acceleration,
                                       angular_velocity);
}

void CarLocalizationTrajectoryBuilder::AddOdometerData(const common::Time time,
                                              const transform::Rigid3d& pose) {
  local_trajectory_builder_.AddOdometerData(time, pose);
}

const mapping::GlobalTrajectoryBuilderInterface::PoseEstimate&
CarLocalizationTrajectoryBuilder::pose_estimate() const {
  return local_trajectory_builder_.pose_estimate();
}

void CarLocalizationTrajectoryBuilder::SetPose(const transform::Rigid3d& pos)
{
  local_trajectory_builder_.SetPose(pos);
}

void CarLocalizationTrajectoryBuilder::GetPose(transform::Rigid3d& pos, double& confidence)
{
  local_trajectory_builder_.GetPose(pos, confidence);
}
}  // namespace mapping_2d
}  // namespace cartographer
