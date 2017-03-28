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

#include "cartographer_android/sensor_bridge.h"

#include "cartographer/kalman_filter/pose_tracker.h"

namespace cartographer_android {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

const string& CheckNoLeadingSlash(const string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/');
  }
  return frame_id;
}

}  // namespace

SensorBridge::SensorBridge(
    const string& tracking_frame, const double lookup_transform_timeout_sec,
    carto::mapping::TrajectoryBuilder* const trajectory_builder)
    : trajectory_builder_(trajectory_builder) {}

void SensorBridge::HandleRangefinder(const string& sensor_id,
                                     const carto::common::Time time,
                                     const string& frame_id,
                                     const carto::sensor::PointCloud& ranges) {
    trajectory_builder_->AddRangefinderData(
        sensor_id, time, Eigen::Vector3f(),
        carto::sensor::TransformPointCloud(ranges,
                                           carto::transform::Rigid3f()));
}

}  // namespace cartographer_android
