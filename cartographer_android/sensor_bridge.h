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

#ifndef CARTOGRAPHER_ROS_SENSOR_BRIDGE_H_
#define CARTOGRAPHER_ROS_SENSOR_BRIDGE_H_

#include "cartographer/mapping/trajectory_builder.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer_android {

// Converts ROS messages into SensorData in tracking frame for the MapBuilder.
class SensorBridge {
 public:
  explicit SensorBridge(
      const string& tracking_frame, double lookup_transform_timeout_sec,
      ::cartographer::mapping::TrajectoryBuilder* trajectory_builder);

  SensorBridge(const SensorBridge&) = delete;
  SensorBridge& operator=(const SensorBridge&) = delete;

 private:
  void HandleRangefinder(const string& sensor_id,
                         const ::cartographer::common::Time time,
                         const string& frame_id,
                         const ::cartographer::sensor::PointCloud& ranges);

  ::cartographer::mapping::TrajectoryBuilder* const trajectory_builder_;
};

}  // namespace cartographer_android

#endif  // CARTOGRAPHER_ROS_SENSOR_BRIDGE_H_
