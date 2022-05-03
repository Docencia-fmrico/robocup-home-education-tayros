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
#include "ros/ros.h"
#include <image_geometry/pinhole_camera_model.h>
#include "taymsgs/person_info.h"
#include "std_msgs/Int32.h"

namespace tayfinder
{

enum{
  PERSON_BUFFER = 4,
};


typedef struct
{
 double x;
 double y;
 double z;
 double roll;
 double pitch;
 double yaw;
 int zone;
 int status;
}PersonPoint3D;

typedef struct
{
	double minX;
	double	maxX;
	double minY;
	double maxY;
	int id;
}Zone;

class PersonLocalizator
{
public:
  PersonLocalizator();
  void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);
  void callback_feedback(const std_msgs::Int32::ConstPtr& id);
  void publish_person_data();

private:
	ros::NodeHandle nh;
	const bool DEBUG = true;

	ros::Publisher person_info_pub_;
	ros::Subscriber feedback_sub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
	darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
	message_filters::Subscriber<sensor_msgs::Image> image_sub;
	message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;
	message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;


	// ---------------- Persons Info --------------- //
	const int EMPTY = -1;
	const int NOT_STUDIED = 0;
	const int STUDIED = 1;
	const float PERSON_RANGE = 1; // metros
	PersonPoint3D person_list[PERSON_BUFFER];
	// ---------------- ------------ --------------- //

	// ----------------- Zones Info ---------------- //
	static const int ZONES_NUMBER = 6;

	const int OUTSIDE = -1;
	Zone zones[ZONES_NUMBER];
	// ---------------- ------------ --------------- //

	void restart_person_status();
	bool is_saved(PersonPoint3D point);
	int get_zone(PersonPoint3D point);
	PersonPoint3D get_3d_map_point();
	
	std::string detectedObject_;
	std::string TopicID;
	std::string workingFrameId_;
	std::string objectTfName_;
};
}  // namespace tayfinder

#endif  
