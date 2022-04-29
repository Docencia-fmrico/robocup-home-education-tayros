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

#ifndef TAYFINDER_PERSON_LOCALIZATOR_H
#define TAYFINDER_PERSON_LOCALIZATOR_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <image_transport/image_transport.h>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <image_geometry/pinhole_camera_model.h>

namespace tayfinder
{

enum{
  PERSON_BUFFER = 4,
};

typedef struct
{
 float x;
 float y; 
 float z;
 int status;
}PersonPoint3D;

class PersonLocalizator
{
public:
  PersonLocalizator();
  void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);
	void callback_cam_info(const sensor_msgs::CameraInfoConstPtr& cam_info);

private:
  ros::NodeHandle nh;
  const bool DEBUG = true;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
  darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
  message_filters::Subscriber<sensor_msgs::Image> image_sub;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;

	const int EMPTY = -1;
	const int NOT_STUDIED = 0;
	const int STUDIED = 1;
	const float PERSON_RANGE = 1; // metros
	PersonPoint3D person_list[PERSON_BUFFER];
	bool cam_info_taked;
	image_geometry::PinholeCameraModel cam_model_;

	void restart_person_status();
	bool is_saved(PersonPoint3D point);
  
  std::string detectedObject_;
  std::string TopicID;
  std::string workingFrameId_;
};
}  // namespace tayfinder

#endif  
