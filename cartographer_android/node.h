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

#ifndef CARTOGRAPHER_ROS_NODE_H_
#define CARTOGRAPHER_ROS_NODE_H_

#include <memory>
#include <vector>

#include "cartographer/common/mutex.h"
#include "cartographer_android/map_builder_bridge.h"
#include "cartographer_android/node_options.h"

namespace cartographer_android {

class Node {
 public:
  Node(const NodeOptions& options);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  void Initialize();

  MapBuilderBridge* map_builder_bridge();

 private:
  void SpinOccupancyGridThreadForever();

  const NodeOptions options_;

  cartographer::common::Mutex mutex_;
  MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);
  int trajectory_id_ = -1;
  std::unordered_set<string> expected_sensor_ids_;

  cartographer::common::Time last_scan_matched_point_cloud_time_ =
      cartographer::common::Time::min();

  std::thread occupancy_grid_thread_;
  bool terminating_ GUARDED_BY(mutex_);
};

}  // namespace cartographer_android

#endif  // CARTOGRAPHER_ROS_NODE_H_
