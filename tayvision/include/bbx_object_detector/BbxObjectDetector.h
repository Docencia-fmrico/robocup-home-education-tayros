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

#ifndef HUMAN_PERCEPTION_BBXDETECTOR_H
#define HUMAN_PERCEPTION_BBXDETECTOR_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Int32.h"
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "std_msgs/String.h"

#include <string>

namespace bbx_object_detector
{

class BbxObjectDetector
{
public:
  BbxObjectDetector();
  void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);
  void callback_activation(const std_msgs::Int32::ConstPtr& activator);

private:
  ros::NodeHandle nh;
  ros::Publisher object_pub_;
  ros::Subscriber activation_sub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
  darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
  message_filters::Subscriber<sensor_msgs::Image> image_depth_sub;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;

  const float CAMERA_XCENTER_ = 340;
  const float MAX_DISTANCE_DIFFERENCE_ = 0.5;
  const float MAX_ANGLE_DIFFERENCE_ = 0.4;

  bool activation_;
  bool object_restart_;
  double current_time_;
  double last_time_;
  const int RESTART_TIME = 3;

  float person_ang_;
  float person_dist_;

  float object_diff_ang_; // Save the ang of the nearest object
  std::string object_name_;


  static const int OBJECTS_NUM = 3;
  std::string object_list[OBJECTS_NUM];
  bool is_searched_object(darknet_ros_msgs::BoundingBox box);

  std::string TopicID;
  std::string workingFrameId_;
  std::string TopicInfoPub;
};
}  // namespace bbx_object_detector

#endif  // HUMAN_PERCEPTION_BBXDETECTOR_H
