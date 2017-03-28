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

#include "cartographer_android/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_android/sensor_bridge.h"
#include "glog/logging.h"

namespace cartographer_android {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

Node::Node(const NodeOptions& options)
    : options_(options), map_builder_bridge_(options_), terminating_(false) {}

Node::~Node() {
  {
    carto::common::MutexLocker lock(&mutex_);
    terminating_ = true;
  }
  if (occupancy_grid_thread_.joinable()) {
    occupancy_grid_thread_.join();
  }
}

void Node::Initialize() {
  carto::common::MutexLocker lock(&mutex_);

  if (options_.map_builder_options.use_trajectory_builder_2d()) {
    occupancy_grid_thread_ =
        std::thread(&Node::SpinOccupancyGridThreadForever, this);
  }
}

MapBuilderBridge* Node::map_builder_bridge() { return &map_builder_bridge_; }

void Node::SpinOccupancyGridThreadForever() {
  for (;;) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    {
      carto::common::MutexLocker lock(&mutex_);
      if (terminating_) {
        return;
      }
    }
    //const auto occupancy_grid = map_builder_bridge_.BuildOccupancyGrid();
    //if (occupancy_grid != nullptr) {
      //occupancy_grid_publisher_.publish(*occupancy_grid);
    //}
  }
}

}  // namespace cartographer_android
