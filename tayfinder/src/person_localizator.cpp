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
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "person_localizator/PersonLocalizator.h"
#include <image_geometry/pinhole_camera_model.h>
#include "taymsgs/person_info.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "person_localizator");
  tayfinder::PersonLocalizator person_localizator;

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    person_localizator.publish_person_data();

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}
