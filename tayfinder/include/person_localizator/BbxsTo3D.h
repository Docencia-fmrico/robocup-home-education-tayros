// Copyright 2022 TayRos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BBXS_TO_3D_TAYFINDER_H
#define BBXS_TO_3D_TAYFINDER_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <string>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

namespace tayfinder
{

class BbxsTo3D
{
public:
  BbxsTo3D();
  void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);
	void callback_cam_info(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info);
private:
  ros::NodeHandle nh;

  ros::Publisher position_pub;

  // Bounding box(darknet)
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
  darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
  message_filters::Subscriber<sensor_msgs::Image> image_depth_sub;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;
  
  // Bbx to 3d point (image geometry(cv))
  typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy_cam;
	image_geometry::PinholeCameraModel cam_model_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub;
  message_filters::Synchronizer<MySyncPolicy_cam> sync_cam;

  int px_;  // Center ground object value
	int py_;
  float distance_;

	bool cam_info_taked;
  tf::TransformBroadcaster tfBroadcaster_;

  std::string detectedObject_;
  std::string ObjectFrameId_;
  std::string workingFrameId_;
};
}  // namespace tayfinder

#endif  // BBXS_TO_3D_TAYFINDER_H
