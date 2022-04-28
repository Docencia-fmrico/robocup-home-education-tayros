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

#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "bbx_color_detection/BbxColorDetector.h"
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "object_bbx_filter/BbxFilter.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "bbx_color_detector");
  bbx_color_detection::BbxColorDetector bbx_detector;
  ros::spin();
  return 0;
}
