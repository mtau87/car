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

#include "cartographer_rviz/submaps_display.h"

#include <hash_set>

#include "OgreResourceGroupManager.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/mutex.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/LoopClosureSubmap.h" 
#include "geometry_msgs/TransformStamped.h"
#include "pluginlib/class_list_macros.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/string_property.h"
#include "glog/logging.h"
#include "eigen_conversions/eigen_msg.h"

namespace cartographer_rviz {

namespace {

constexpr int kMaxOnGoingRequestsPerTrajectory = 6;
constexpr char kMaterialsDirectory[] = "/ogre_media/materials";
constexpr char kGlsl120Directory[] = "/glsl120";
constexpr char kScriptsDirectory[] = "/scripts";
constexpr char kDefaultMapFrame[] = "map";
constexpr char kDefaultTrackingFrame[] = "base_link";
constexpr char kDefaultSubmapQueryServiceName[] = "/submap_query";
::ros::Publisher* g_pubLoopClosureSubmap = NULL;

std::string GetMarkerIdByName(std::string strMarkerName)
{
  int index = strMarkerName.find("_");
  return strMarkerName.substr(index + 1);
}

// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      {
        LOG(WARNING) << "Mouse click: " << feedback->marker_name;
        //ROS_INFO_STREAM( "mouse click." );
      }
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      {
        LOG(INFO) << "Menu click: " << feedback->marker_name;
        std::string strId = GetMarkerIdByName(feedback->marker_name);
        std::stringstream ss;
        ss << strId;
        int nId;
        ss >> nId;
        ::cartographer_ros_msgs::LoopClosureSubmap msg;
        msg.submap_index = nId;
        g_pubLoopClosureSubmap->publish(msg);
      }
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      // ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      // ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  //server_->applyChanges();
}
// %EndTag(processFeedback)%

// %Tag(Box)%
visualization_msgs::Marker makeBox( visualization_msgs::InteractiveMarker &msg, float fSize = 0.45)
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = msg.scale * fSize;
  marker.scale.y = msg.scale * fSize;
  marker.scale.z = msg.scale * fSize;
  marker.color.r = 0.7;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  return marker;
}
// %EndTag(Box)%

}  // namespace

SubmapsDisplay::SubmapsDisplay() : tf_listener_(tf_buffer_) {
  submap_query_service_property_ = new ::rviz::StringProperty(
      "Submap query service", kDefaultSubmapQueryServiceName,
      "Submap query service to connect to.", this, SLOT(Reset()));
  map_frame_property_ = new ::rviz::StringProperty(
      "Map frame", kDefaultMapFrame, "Map frame, used for fading out submaps.",
      this);
  tracking_frame_property_ = new ::rviz::StringProperty(
      "Tracking frame", kDefaultTrackingFrame,
      "Tracking frame, used for fading out submaps.", this);
  client_ = update_nh_.serviceClient<::cartographer_ros_msgs::SubmapQuery>("");
  const std::string package_path = ::ros::package::getPath(ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      package_path + kMaterialsDirectory, "FileSystem", ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      package_path + kMaterialsDirectory + kGlsl120Directory, "FileSystem",
      ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      package_path + kMaterialsDirectory + kScriptsDirectory, "FileSystem",
      ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  trajectory_id_ = -1;
  submap_index_ = -1;

  server_.reset( new interactive_markers::InteractiveMarkerServer("submap_list","",false) );
  
  connect(this, SIGNAL(RequestSucceeded()), this, SLOT(UpdateSceneNode()));

  menu_handler_.insert( "Find Loop Closure", &processFeedback );

  loop_closure_submap_publisher_ =
  node_handle_.advertise<::cartographer_ros_msgs::LoopClosureSubmap>(
      kLoopClosureSubmapTopic, 1);
  g_pubLoopClosureSubmap = &loop_closure_submap_publisher_;
}

SubmapsDisplay::~SubmapsDisplay() {
  server_.reset();
  client_.shutdown(); 
}

void SubmapsDisplay::Reset() { reset(); }

void SubmapsDisplay::CreateClient() {
  client_ = update_nh_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(
      submap_query_service_property_->getStdString());
}

void SubmapsDisplay::onInitialize() {
  MFDClass::onInitialize();
  CreateClient();
}

void SubmapsDisplay::reset() {
  MFDClass::reset();
  ::cartographer::common::MutexLocker locker(&mutex_);
  client_.shutdown();
  trajectories_.clear();
  CreateClient();
}

void SubmapsDisplay::processMessage(
    const ::cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
  ::cartographer::common::MutexLocker locker(&mutex_);
  //LOG(INFO) << "Trajectory size: " << msg->trajectory.size();
  for (size_t trajectory_id = 0; trajectory_id < msg->trajectory.size();
       ++trajectory_id) {
    if (trajectory_id >= trajectories_.size()) {
      trajectories_.emplace_back();
    }
    auto& trajectory = trajectories_[trajectory_id];
    const std::vector<::cartographer_ros_msgs::SubmapEntry>& submap_entries =
        msg->trajectory[trajectory_id].submap;
    //LOG(INFO) << "submap_entries size: " << submap_entries.size();
    for (size_t submap_index = 0; submap_index < submap_entries.size();
         ++submap_index) {
      if (submap_index >= trajectory.size()) {
        trajectory.push_back(
            ::cartographer::common::make_unique<DrawableSubmap>(
                trajectory_id, submap_index, context_->getSceneManager()));
      }

      trajectory[submap_index]->Update(msg->header,
                                       submap_entries[submap_index],
                                       context_->getFrameManager());
      // trajectory_id_ = trajectory_id;
      // submap_index_ = submap_index;
      Ogre::Vector3 vecSubmapPos = trajectories_[trajectory_id][submap_index]->GetPosition();
      tf::Vector3 position;
      position = tf::Vector3(vecSubmapPos.x, vecSubmapPos.y, 0.1f);
      //LOG(INFO) << "--------------------submap: " << submap_index_ << ":" << position[0] << ", " << position[1]; 
      std::string strMarkerName;
      std::ostringstream strStream;    
      strStream << submap_index;   
      std::string strIdx = strStream.str();        
      strMarkerName = "submap_" + strIdx;
      visualization_msgs::InteractiveMarker int_marker;
      if(server_->get(strMarkerName, int_marker))
      {
        geometry_msgs::Pose pose = int_marker.pose;
        pose.position.x = vecSubmapPos.x;
        pose.position.y = vecSubmapPos.y;
        server_->setPose(strMarkerName, pose);
      }
      else
      {
        MakeMarker(submap_index, position, 1.0);
      }
      // query_in_progress_ = true;
      // rpc_request_future_ = std::async(std::launch::async, [this]() {
      //   ::cartographer_ros_msgs::SubmapQuery srv;
      //   srv.request.trajectory_id = trajectory_id_;
      //   srv.request.submap_index = submap_index_;
      //   if (client_.call(srv)) {
      //     // We emit a signal to update in the right thread, and pass via the
      //     // 'response_' member to simplify the signal-slot connection slightly.
      //     ::cartographer::common::MutexLocker locker(&mutex_);
      //     response_ = srv.response;
      //     Q_EMIT RequestSucceeded();
      //     tf::poseMsgToEigen(response_.slice_pose, slice_pose_);

      //     Ogre::Vector3 vecSubmapPos = trajectories_[trajectory_id_][submap_index_]->GetPosition();
      //     tf::Vector3 position;
      //     position = tf::Vector3(vecSubmapPos.x, vecSubmapPos.y, 0.1f);
      //     //LOG(INFO) << "--------------------submap: " << submap_index_ << ":" << position[0] << ", " << position[1];  
      //     if(pushFlag)             
      //     {
      //       MakeMarker(submap_index_, position, 1.0);
      //     }
      //     else
      //     {

      //       if(server_->get(strMarkerName, int_marker)
      //       {
      //         InteractiveMarker int_marker;
      //       }
      //       server_->setPose("", )
      //     }
                       
      //   } else {
      //     ::cartographer::common::MutexLocker locker(&mutex_);
      //     query_in_progress_ = false;
      //   }
      // });      
    }
  }
}

void SubmapsDisplay::update(const float wall_dt, const float ros_dt) {
  // Update the fading by z distance.
  try {
    const ::geometry_msgs::TransformStamped transform_stamped =
        tf_buffer_.lookupTransform(map_frame_property_->getStdString(),
                                   tracking_frame_property_->getStdString(),
                                   ros::Time(0) /* latest */);
    ::cartographer::common::MutexLocker locker(&mutex_);
    for (auto& trajectory : trajectories_) {
      for (auto& submap : trajectory) {
        submap->SetAlpha(transform_stamped.transform.translation.z);
      }
    }
  } catch (const tf2::TransformException& ex) {
    ROS_WARN("Could not compute submap fading: %s", ex.what());
  }

  // Schedule fetching of new submap textures.
  for (const auto& trajectory : trajectories_) {
    int num_ongoing_requests = 0;
    for (const auto& submap : trajectory) {
      if (submap->QueryInProgress()) {
        ++num_ongoing_requests;
      }
    }
    for (int submap_index = trajectory.size() - 1;
         submap_index >= 0 &&
         num_ongoing_requests < kMaxOnGoingRequestsPerTrajectory;
         --submap_index) {
      if (trajectory[submap_index]->MaybeFetchTexture(&client_)) {
        ++num_ongoing_requests;
      }
    }
  }
}

// %Tag()%
void SubmapsDisplay::MakeMarker(int nId, const tf::Vector3& position, float fSize)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;
  std::ostringstream strStream;    
  strStream << nId;   
  std::string strId = strStream.str();
  int_marker.name = "submap_" + strId;
  //int_marker.description = "Button\n(Left Click)";

  visualization_msgs::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  visualization_msgs::Marker marker = makeBox(int_marker, fSize);
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  //server_->setCallback(int_marker.name, &processFeedback);
  menu_handler_.apply( *server_, int_marker.name );
  server_->applyChanges();
}
// %EndTag()%

void SubmapsDisplay::UpdateSceneNode() {
  ROS_INFO("update submap!");
  query_in_progress_ = false;
}

}  // namespace cartographer_rviz

PLUGINLIB_EXPORT_CLASS(cartographer_rviz::SubmapsDisplay, ::rviz::Display)
