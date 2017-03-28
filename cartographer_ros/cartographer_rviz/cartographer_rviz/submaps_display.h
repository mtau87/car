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

#ifndef CARTOGRAPHER_RVIZ_SRC_SUBMAPS_DISPLAY_H_
#define CARTOGRAPHER_RVIZ_SRC_SUBMAPS_DISPLAY_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_rviz/drawable_submap.h"
#include "rviz/message_filter_display.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

namespace cartographer_rviz {

constexpr char kLoopClosureSubmapTopic[] = "loop_closure_submap";

// RViz plugin used for displaying maps which are represented by a collection of
// submaps.
//
// We show an X-ray view of the map which is achieved by shipping textures for
// every submap containing pre-multiplied alpha and grayscale values, these are
// then alpha blended together.
class SubmapsDisplay
    : public ::rviz::MessageFilterDisplay<::cartographer_ros_msgs::SubmapList> {
  Q_OBJECT

 public:
  SubmapsDisplay();
  ~SubmapsDisplay() override;

  SubmapsDisplay(const SubmapsDisplay&) = delete;
  SubmapsDisplay& operator=(const SubmapsDisplay&) = delete;

  void MakeMarker(int nId, const tf::Vector3& position, float fSize = 0.45); 

 Q_SIGNALS:
  // RPC request succeeded.
  void RequestSucceeded();

 private Q_SLOTS:
  // Callback when an rpc request succeeded.
  void UpdateSceneNode();

 private Q_SLOTS:
  void Reset();

 private:
  void CreateClient();

  // These are called by RViz and therefore do not adhere to the style guide.
  void onInitialize() override;
  void reset() override;
  void processMessage(
      const ::cartographer_ros_msgs::SubmapList::ConstPtr& msg) override;
  void update(float wall_dt, float ros_dt) override;

  ::tf2_ros::Buffer tf_buffer_;
  ::tf2_ros::TransformListener tf_listener_;
  ros::ServiceClient client_;
  ::rviz::StringProperty* submap_query_service_property_;
  ::rviz::StringProperty* map_frame_property_;
  ::rviz::StringProperty* tracking_frame_property_;
  using Trajectory = std::vector<std::unique_ptr<DrawableSubmap>>;
  std::vector<Trajectory> trajectories_ GUARDED_BY(mutex_);
  ::cartographer_ros_msgs::SubmapQuery::Response response_ GUARDED_BY(mutex_);
  Eigen::Affine3d slice_pose_ GUARDED_BY(mutex_);  
  std::chrono::milliseconds last_query_timestamp_ GUARDED_BY(mutex_);
  bool query_in_progress_ = false GUARDED_BY(mutex_);  
  std::future<void> rpc_request_future_;  
  ::cartographer::common::Mutex mutex_;

  int trajectory_id_;
  int submap_index_;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;   
  interactive_markers::MenuHandler menu_handler_;
  ::ros::NodeHandle node_handle_;  
  ::ros::Publisher loop_closure_submap_publisher_;  
};

}  // namespace cartographer_rviz

#endif  // CARTOGRAPHER_RVIZ_SRC_SUBMAPS_DISPLAY_H_
