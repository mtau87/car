/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseArray.h>
#include "cartographer_ros_msgs/TrajectoryNodes.h"

#include <math.h>
#include <string>
#include <hash_set>

using namespace __gnu_cxx;
using namespace visualization_msgs;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
std_msgs::ColorRGBA colorBlue;
std_msgs::ColorRGBA colorGrey;   
hash_set<int> g_setSelectedMarkers;
ros::ServiceClient g_clientServiceTrajectoryNodes;

// %EndTag(vars)%

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
void makeMarker(int nId, const tf::Vector3& position, float fSize);

bool GetMarkerStateById(std::string strName)
{
  int index = strName.find("_");
  std::string strStatus = strName.substr(index + 1, strName.size());
  if(strStatus == "checked")
  {
    return true;
  }
  return false;
}

std::string GetMarkerIdByName(std::string strMarkerName)
{
  int index = strMarkerName.find("_");
  return strMarkerName.substr(0, index);
}

std::string GetMarkerNameStringByState(std::string strName, bool bChecked)
{
  int index = strName.find("_");
  std::string strId = strName.substr(0, index); 
  std::string strStatus = "unchecked";
  if(bChecked)
  {
    strStatus = "checked";
  }
  return strId + "_" + strStatus;
}

void MaintainSelectedSet(std::string strMarkerId, bool bChecked)
{
  int nId = atoi(strMarkerId.c_str());
  if(bChecked)
  {
    g_setSelectedMarkers.insert(nId);
  }
  else
  {
    g_setSelectedMarkers.erase(nId);
  }
}

bool SetMarkerState(std::string strMarkerName, bool bChecked)
{
  InteractiveMarker int_marker;
  std_msgs::ColorRGBA color = colorGrey;
  float fScale = 1.0/1.2;
  if(bChecked)
  {
    color = colorBlue;
    fScale = 1.2;
  }
  if(server->get(strMarkerName, int_marker))
  {
    std::string strNewMarkerName = GetMarkerNameStringByState(strMarkerName, bChecked);
    Marker& marker = int_marker.controls[0].markers[0];
    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = color.a;
    marker.scale.x *= fScale;
    marker.scale.y *= fScale;
    marker.scale.z *= fScale;


    server->erase(strMarkerName);
    int_marker.name = strNewMarkerName;
    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);   
    MaintainSelectedSet(GetMarkerIdByName(int_marker.name), bChecked); 
    std::ostringstream s;    
    s << g_setSelectedMarkers.size();    
    ROS_INFO_STREAM( "selected marker size: " << s.str() << "\n");        
    return true;
  }
  return false;
}

// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg, float fSize = 0.45)
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * fSize;
  marker.scale.y = msg.scale * fSize;
  marker.scale.z = msg.scale * fSize;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}
// %EndTag(Box)%

// %Tag(frameCallback)%
void frameCallback(const ros::TimerEvent&)
{
  //std::cout << "heart beat123" << std::endl;
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;

  tf::Transform t;

  ros::Time time = ros::Time::now();

  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0 /*sin(float(counter)/140.0) * 2.0*/));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));

  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(0.0, 0.0 /*float(counter)/140.0*/, 0.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "rotating_frame"));
  //std::cout << "heart beat" << std::endl;
  counter++;
}
// %EndTag(frameCallback)%

// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  // std::ostringstream s;
  // s << "Feedback from marker '" << feedback->marker_name << "' "
  //     << " / control '" << feedback->control_name << "'";

  // std::ostringstream mouse_point_ss;
  // if( feedback->mouse_point_valid )
  // {
  //   mouse_point_ss << " at " << feedback->mouse_point.x
  //                  << ", " << feedback->mouse_point.y
  //                  << ", " << feedback->mouse_point.z
  //                  << " in frame " << feedback->header.frame_id;
  // }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      // ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      {
          ::cartographer_ros_msgs::TrajectoryNodes srv;
          if (g_clientServiceTrajectoryNodes.call(srv)) {
            std::cout << srv.response.poses.poses.size() << std::endl;
            for (int i = 0; i < srv.response.poses.poses.size(); ++i)
            {
              tf::Vector3 position;
              position = tf::Vector3(srv.response.poses.poses[i].position.x, srv.response.poses.poses[i].position.y, srv.response.poses.poses[i].position.z);
              makeMarker(i, position, 0.3);  
            }

          }

        if(GetMarkerStateById(feedback->marker_name))
        {
          SetMarkerState(feedback->marker_name, false);
        }
        else
        {
          SetMarkerState(feedback->marker_name, true);
        }
      }
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      // ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      // ROS_INFO_STREAM( s.str() << ": pose changed"
      //     << "\nposition = "
      //     << feedback->pose.position.x
      //     << ", " << feedback->pose.position.y
      //     << ", " << feedback->pose.position.z
      //     << "\norientation = "
      //     << feedback->pose.orientation.w
      //     << ", " << feedback->pose.orientation.x
      //     << ", " << feedback->pose.orientation.y
      //     << ", " << feedback->pose.orientation.z
      //     << "\nframe: " << feedback->header.frame_id
      //     << " time: " << feedback->header.stamp.sec << "sec, "
      //     << feedback->header.stamp.nsec << " nsec" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      // ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      // ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  server->applyChanges();
}
// %EndTag(processFeedback)%

double rand( double min, double max )
{
  double t = (double)rand() / (double)RAND_MAX;
  return min + t*(max-min);
}

void saveMarker( InteractiveMarker int_marker )
{
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

////////////////////////////////////////////////////////////////////////////////////

// %Tag()%
void makeMarker(int nId, const tf::Vector3& position, float fSize = 0.45)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;
  std::ostringstream strStream;    
  strStream << nId;   
  std::string strId = strStream.str();
  int_marker.name = strId + "_unchecked";
  //int_marker.description = "Button\n(Left Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  Marker marker = makeBox(int_marker, fSize);
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag()%

void CarLoopClosureCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  static int nIdCounter = 0;
  std::ostringstream strStream;    
  strStream << "(" << msg->poses[0].position.x << ", " << msg->poses[0].position.y << ")\n";   
  ROS_INFO_STREAM(strStream.str());  
  tf::Vector3 position;
  position = tf::Vector3(msg->poses[0].position.x, msg->poses[0].position.y, 0);
  makeMarker(nIdCounter++, position, 0.3);  
  server->applyChanges();
}

// %Tag(main)%
int main(int argc, char** argv)
{
  colorBlue.r = 0.0;
  colorBlue.g = 0.0;
  colorBlue.b = 0.7;
  colorBlue.a = 1.0;

  colorGrey.r = 0.5;
  colorGrey.g = 0.5;
  colorGrey.b = 0.5;
  colorGrey.a = 1.0; 

  ros::init(argc, argv, "basic_controls");
  ros::NodeHandle n;
  std::cout << "init" << std::endl;
  ros::Subscriber subCarLoopClosure = n.subscribe("car_loop_closure", 1000, CarLoopClosureCallback);
  g_clientServiceTrajectoryNodes = n.serviceClient< ::cartographer_ros_msgs::TrajectoryNodes>("trajectory_nodes_query");

  // create a timer to update the published transforms
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

  ros::Duration(0.1).sleep();

  // menu_handler.insert( "First Entry", &processFeedback );
  // menu_handler.insert( "Second Entry", &processFeedback );
  // interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Submenu" );
  // menu_handler.insert( sub_menu_handle, "First Entry", &processFeedback );
  // menu_handler.insert( sub_menu_handle, "Second Entry", &processFeedback );

  int nSizeX = 5;
  int nSizeY = 5;
  for(int i = 0; i < nSizeX; i++)
  {
    for(int j = 0; j < nSizeY; j++)
    {
      tf::Vector3 position;
      position = tf::Vector3(i * 1.0, j * 1.0, 0);
      int id = i * nSizeY + j + 100000;
      makeMarker(id, position, 0.3);
    }   
  }

  server->applyChanges();

  ros::spin();

  server.reset();
}
// %EndTag(main)%
