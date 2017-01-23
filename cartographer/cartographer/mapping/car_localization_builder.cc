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

#include "cartographer/mapping/car_localization_builder.h"

#include <cmath>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/car_collated_trajectory_builder.h"
#include "cartographer/mapping_2d/car_localization_trajectory_builder.h"
#include "cartographer/mapping_3d/global_trajectory_builder.h"
#include "cartographer/mapping_3d/local_trajectory_builder_options.h"
#include "cartographer/sensor/laser.h"
#include "cartographer/sensor/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

CarLocalizationBuilder::CarLocalizationBuilder(
    const proto::MapBuilderOptions& options,
    const mapping_2d::ProbabilityGrid& probability_grid)
    : options_(options), thread_pool_(options.num_background_threads()), probability_grid_(probability_grid) {
}

CarLocalizationBuilder::~CarLocalizationBuilder() {}

int CarLocalizationBuilder::AddTrajectoryBuilder(
    const std::unordered_set<string>& expected_sensor_ids) {
  const int trajectory_id = trajectory_builders_.size();
  trajectory_builders_.push_back(
      common::make_unique<CarCollatedTrajectoryBuilder>(
          &sensor_collator_, trajectory_id, expected_sensor_ids,
          common::make_unique<mapping_2d::CarLocalizationTrajectoryBuilder>(
              options_.trajectory_builder_2d_options(),
              probability_grid_)));
  trajectory_ids_.emplace(trajectory_builders_.back()->submaps(),
                          trajectory_id);
  return trajectory_id;
}

TrajectoryBuilder* CarLocalizationBuilder::GetTrajectoryBuilder(
    const int trajectory_id) const {
  return trajectory_builders_.at(trajectory_id).get();
}

void CarLocalizationBuilder::FinishTrajectory(const int trajectory_id) {
  sensor_collator_.FinishTrajectory(trajectory_id);
}

int CarLocalizationBuilder::GetBlockingTrajectoryId() const {
  return sensor_collator_.GetBlockingTrajectoryId();
}

int CarLocalizationBuilder::GetTrajectoryId(const Submaps* const trajectory) const {
  const auto trajectory_id = trajectory_ids_.find(trajectory);
  CHECK(trajectory_id != trajectory_ids_.end());
  return trajectory_id->second;
}

int CarLocalizationBuilder::num_trajectory_builders() const {
  return trajectory_builders_.size();
}

}  // namespace mapping
}  // namespace cartographer
