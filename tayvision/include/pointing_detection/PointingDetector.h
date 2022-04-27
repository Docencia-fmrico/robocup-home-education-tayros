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

#ifndef TAYVISION_POINTINGDETECTOR_H
#define TAYVISION_POINTINGDETECTOR_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <string>

namespace tayvision
{

class PointingDetector
{
public:
  PointingDetector();
  void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);
  
private:
  ros::NodeHandle nh;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
  darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
  message_filters::Subscriber<sensor_msgs::Image> image_depth_sub;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;

	// --------------------- MOVEMENT FILTER --------------------- //
	const float CAMERA_XCENTER_ = 340;
  const float MAX_DISTANCE_DIFFERENCE_ = 0.2;
  const float MAX_ANGLE_DIFFERENCE_ = 0.1;
  const int RESTART_TICKS_ = 100;

  float last_distance_;  // Save last distance measured
  float last_angle_;
  int restart_counter;
  bool bbx_restart;
	// ------------------- ------------------- -------------------- //


	// --------------------- POINTING --------------------- //
  const int MIN_PX_CAHNGE = 30;  // the minimun value to consider the person pointing
  const int POINTING_TICKS = 100;
  const int NO_POINTING = -1;
  const int POINTING_RIGHT = 0;  // In function of robot position
  const int POINTING_LEFT = 1;

  int last_px_min_;  // Save last distance measured(to calculate pointing direction)
  int last_px_max_; 
  bool pointing_activation;
  int pointing_ticks_counter;
  // --------------------- -------- --------------------- //

  std::string detectedObject_;
  std::string TopicID;
  std::string workingFrameId_;
};
}  // namespace tayvision

#endif  // TAYVISION_POINTINGDETECTOR_H
