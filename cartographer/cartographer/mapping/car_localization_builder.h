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

#ifndef CARTOGRAPHER_MAPPING_CAR_LOCALIZATION_BUILDER_H_
#define CARTOGRAPHER_MAPPING_CAR_LOCALIZATION_BUILDER_H_

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/sparse_pose_graph.h"
#include "cartographer/mapping_3d/sparse_pose_graph.h"
#include "cartographer/sensor/collator.h"

namespace cartographer {
namespace mapping {

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a SparsePoseGraph for loop closure.
class CarLocalizationBuilder {
 public:
  CarLocalizationBuilder(const proto::MapBuilderOptions& options,
             const mapping_2d::ProbabilityGrid& probability_grid);
  ~CarLocalizationBuilder();

  CarLocalizationBuilder(const CarLocalizationBuilder&) = delete;
  CarLocalizationBuilder& operator=(const CarLocalizationBuilder&) = delete;

  // Create a new trajectory and return its index.
  int AddTrajectoryBuilder(
      const std::unordered_set<string>& expected_sensor_ids);

  // Returns the TrajectoryBuilder corresponding to the specified
  // 'trajectory_id'.
  mapping::TrajectoryBuilder* GetTrajectoryBuilder(int trajectory_id) const;

  // Marks the TrajectoryBuilder corresponding to 'trajectory_id' as finished,
  // i.e. no further sensor data is expected.
  void FinishTrajectory(int trajectory_id);

  // Must only be called if at least one unfinished trajectory exists. Returns
  // the ID of the trajectory that needs more data before the MapBuilder is
  // unblocked.
  int GetBlockingTrajectoryId() const;

  // Returns the trajectory ID for 'trajectory'.
  int GetTrajectoryId(const mapping::Submaps* trajectory) const;

  int num_trajectory_builders() const;

 private:
  const proto::MapBuilderOptions options_;
  common::ThreadPool thread_pool_;

  sensor::Collator sensor_collator_;
  std::vector<std::unique_ptr<mapping::TrajectoryBuilder>> trajectory_builders_;
  std::unordered_map<const mapping::Submaps*, int> trajectory_ids_;
  const mapping_2d::ProbabilityGrid& probability_grid_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_CAR_LOCALIZATION_BUILDER_H_
