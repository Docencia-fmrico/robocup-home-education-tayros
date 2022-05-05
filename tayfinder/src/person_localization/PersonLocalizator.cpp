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
#include <unistd.h>
#include <string.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "taymsgs/person_info.h"

namespace tayfinder
{

PersonLocalizator::PersonLocalizator():
  workingFrameId_("/base_footprint"),
  detectedObject_("person"),
  objectTfName_("bbx_person"),
  image_sub(nh, "/camera/depth/image_raw", 1), // /camera/depth/image_raw"
  bbx_sub(nh, "/darknet_ros/bounding_boxes", 1),
  sync_bbx(MySyncPolicy_bbx(10), image_sub, bbx_sub)
  {
    sync_bbx.registerCallback(boost::bind(&PersonLocalizator::callback_bbx, this, _1, _2));
    feedback_sub_ = nh.subscribe<std_msgs::Int32>("/tayros/person/feedback", 10, &PersonLocalizator::callback_feedback, this);
    person_info_pub_ = nh.advertise<taymsgs::person_info>("/tayros/person_info", 1);
    restart_person_status();
    
    zones[0].id = 1;
    zones[0].minX = -2.0834310054779053;
    zones[0].maxX = 3.23098087310791;
    zones[0].minY = -1.6;
    zones[0].maxY = 9.095319747924805;

    zones[1].id = 2;
    zones[1].minX = -2.0834310054779053;
    zones[1].maxX = 3.23098087310791;
    zones[1].minY = -1.6;
    zones[1].maxY = 9.095319747924805;

    zones[2].id = 3;
    zones[2].minX = -2.0834310054779053;
    zones[2].maxX = 3.23098087310791;
    zones[2].minY = -1.6;
    zones[2].maxY = 9.095319747924805;

    zones[3].id = 4;
    zones[3].minX = -2.0834310054779053;
    zones[3].maxX = 3.23098087310791;
    zones[3].minY = -1.6;
    zones[3].maxY = 9.095319747924805;

    zones[4].id = 5;
    zones[4].minX = -2.0834310054779053;
    zones[4].maxX = 3.23098087310791;
    zones[4].minY = -1.6;
    zones[4].maxY = 9.095319747924805;

    zones[5].id = 6;
    zones[5].minX = -2.0834310054779053;
    zones[5].maxX = 3.23098087310791;
    zones[5].minY = -1.6;
    zones[5].maxY = 9.095319747924805;

    /*
    zones[1].id = 2;
    zones[1].minX = -2.674506664276123;
    zones[1].maxX = -0.052829861640930176;
    zones[1].minY = 7.832345962524414;
    zones[1].maxY = 9.029717445373535;


    zones[2].id = 3;
    zones[2].minX = -2.4512338638305664;
    zones[2].maxX = -1.3018546104431152;
    zones[2].minY = 5.100671291351318;
    zones[2].maxY = 8.195911407470703;

    zones[3].id = 4;
    zones[3].minX = -2.495957374572754;
    zones[3].maxX = -1.3095178604125977;
    zones[3].minY = 2.7130393981933594;
    zones[3].maxY = 5.119590759277344;

    zones[4].id = 5;
    zones[4].minX = -1.5878496170043945;
    zones[4].maxX = -0.42526817321777344;
    zones[4].minY = 1.3998517990112305;
    zones[4].maxY = 3.2975192070007324;


    zones[5].id = 6;
    zones[5].minX = 1.1362266540527344;
    zones[5].maxX = 3.0820064544677734;
    zones[5].minY = 1.3447916507720947;
    zones[5].maxY = 2.8056538105010986;
    */

  }

void PersonLocalizator::callback_bbx(const sensor_msgs::ImageConstPtr& image,
  const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
  cv_bridge::CvImagePtr img_depth_ptr;

  try
  {
    img_depth_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }


	for (const auto & box : boxes->bounding_boxes){

		PersonPoint3D current_person;
    restart_person_data(current_person);
        
		if (box.Class == detectedObject_ )
		{
			current_person = get_3d_map_point();
      std::cout << current_person.x << current_person.y << current_person.y << std::endl;
			int zone = get_zone(current_person);
			if(zone == OUTSIDE){
				return;
			}

			current_person.zone = zone;

			if(!is_saved(current_person)){
				std::cout << "Saving new person" << std::endl;
				for(int i = 0; i < PERSON_BUFFER; i++){
					if(person_list[i].status == EMPTY){
						person_list[i] = current_person;
						person_list[i].status = NOT_STUDIED;
						break;
					}
				}
			}

			// DEBUG 
			for(int i = 0; i < PERSON_BUFFER; i++){
				if(is_saved(person_list[i]) && person_list[i].status != EMPTY){
					std::cout << "**********PERSON " << i << "DATA***************" << std::endl;
					std::cout << "X: " << person_list[i].x << " Y: " << person_list[i].y << " Z: " << person_list[i].z << std::endl;
					std::cout << "Roll: " << person_list[i].roll << " PITCH: " << person_list[i].pitch << " YAW: " << person_list[i].yaw << std::endl;
					std::cout << "ZONE: " << person_list[i].zone << " STATUS: " << person_list[i].status << std::endl;
					std::cout << "*************************************" << std::endl;
				}
			}
		}
	}
}


void 
PersonLocalizator::callback_feedback(const std_msgs::Int32::ConstPtr& id){
  person_list[id->data].status = STUDIED;
}


void 
PersonLocalizator::publish_person_data(){

  taymsgs::person_info current_person;

  for(int i = 0; i < PERSON_BUFFER; i++){
    if(person_list[i].status != EMPTY && person_list[i].status == NOT_STUDIED){
        
      current_person.position.target_pose.header.frame_id = "map";
      current_person.position.target_pose.pose.position.x = person_list[i].x;
      current_person.position.target_pose.pose.position.y = person_list[i].y;
      current_person.position.target_pose.pose.position.z = person_list[i].z;
      current_person.position.target_pose.pose.orientation.x = 0;
      current_person.position.target_pose.pose.orientation.y = 0;
      current_person.position.target_pose.pose.orientation.z = 0;
      current_person.position.target_pose.pose.orientation.w = 1.0;
      
      current_person.id = i;
      current_person.zone = person_list[i].zone;
      person_info_pub_.publish(current_person);
      return;
    }
  }
  return;
}


void 
PersonLocalizator::restart_person_status(){

    for(int i = 0; i < PERSON_BUFFER; i++){
        person_list[i].status = EMPTY;
        person_list[i].x= 100;
        person_list[i].y = 100;
        person_list[i].z = 100;
        person_list[i].pitch= 100;
        person_list[i].roll= 100;
        person_list[i].yaw = 100;
        person_list[i].zone = -1;
    }
    return;
}

void 
PersonLocalizator::restart_person_data(PersonPoint3D point){

  point.x = 100;
  point.y = 100;
  point.z = 100;
  point.pitch = 100;
  point.roll = 100;
  point.yaw = 100;
  point.zone = -1;
  point.status = EMPTY;
}

bool 
PersonLocalizator::is_saved(PersonPoint3D point){
    
    float x_diff = 0;
    float y_diff = 0;

    for(int i = 0; i < PERSON_BUFFER; i++){
        x_diff = abs(point.x - person_list[i].x);
        y_diff = abs(point.y - person_list[i].y);
        if(x_diff < PERSON_RANGE && y_diff < PERSON_RANGE){
            return true;
        }
    }
    return false;
}

int 
PersonLocalizator::get_zone(PersonPoint3D point){

	for(int i = 0; i < ZONES_NUMBER; i++){
		if (point.x > zones[i].minX && point.x < zones[i].maxX 
		&& point.y > zones[i].minY && point.y < zones[i].maxY){
			std::cout << "Person Found in: " << zones[i].id << std::endl;
			return zones[i].id;
		}
	}
	std::cout << "Person Found OUTSIDE" << std::endl;
	return OUTSIDE;
}

PersonPoint3D
PersonLocalizator::get_3d_map_point()
{
	
	tf2_ros::Buffer buffer;
	tf2_ros::TransformListener listener(buffer);

	geometry_msgs::TransformStamped map2odommsg;
    tf2::Stamped<tf2::Transform> map2odom;
    geometry_msgs::TransformStamped odom2bfmsg;
    tf2::Stamped<tf2::Transform> odom2bf;
    geometry_msgs::TransformStamped bf2person_msg;
    tf2::Stamped<tf2::Transform> bf2person;

	PersonPoint3D result_point;
    tf2::Transform map2person_;
	double yaw;
	double pitch;
	double roll;
	std::string error;


  if (buffer.canTransform("map", "odom", ros::Time(0), ros::Duration(1), &error))
  {
  
    map2odommsg = buffer.lookupTransform("map", "odom", ros::Time(0));
    tf2::fromMsg(map2odommsg, map2odom);
    tf2::Matrix3x3(map2odom.getRotation()).getRPY(roll, pitch, yaw);
  }
  else
  {
    //ROS_ERROR("%s", error.c_str());
  }

  if (buffer.canTransform("odom", "base_footprint", ros::Time(0), ros::Duration(1), &error))
  {
    odom2bfmsg = buffer.lookupTransform("odom", "base_footprint", ros::Time(0));
    tf2::fromMsg(odom2bfmsg, odom2bf);
    tf2::Matrix3x3(odom2bf.getRotation()).getRPY(roll, pitch, yaw);
  }
  else
  {
    //ROS_ERROR("%s", error.c_str());
  }

  if (buffer.canTransform("base_footprint", objectTfName_, ros::Time(0), ros::Duration(1), &error))
  {
    bf2person_msg = buffer.lookupTransform("base_footprint", objectTfName_, ros::Time(0));
    tf2::fromMsg(bf2person_msg, bf2person);
    tf2::Matrix3x3(bf2person.getRotation()).getRPY(roll, pitch, yaw);
  }

  else
  {
    //ROS_ERROR("%s", error.c_str());
  }

  ROS_INFO("TRANSFORMATION DONE");
  map2person_ = map2odom * odom2bf * bf2person;

  result_point.x = map2person_.getOrigin().x();
  result_point.y = map2person_.getOrigin().y();
  result_point.z = map2person_.getOrigin().z();

  tf2::Matrix3x3(map2person_.getRotation()).getRPY(result_point.roll, result_point.pitch, result_point.yaw);

  return result_point;
}


}  // namespace tayfinder

