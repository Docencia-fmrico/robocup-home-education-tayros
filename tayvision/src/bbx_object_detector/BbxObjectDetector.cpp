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
#include "std_msgs/Int32.h"
#include <ros/ros.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "bbx_object_detector/BbxObjectDetector.h"
#include "std_msgs/String.h"
#include <string.h>


namespace bbx_object_detector
{

BbxObjectDetector::BbxObjectDetector()
: nh(),
  workingFrameId_("/base_footprint"),
  TopicID("/visual_behavior/person/position"),
  TopicInfoPub("/tayvision/person/object_info"),
  image_depth_sub(nh, "camera/rgb/image_raw", 1),
  bbx_sub(nh, "/darknet_ros/bounding_boxes", 1),
  sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub)
  {
    sync_bbx.registerCallback(boost::bind(&BbxObjectDetector::callback_bbx, this, _1, _2));
    object_pub_ = nh.advertise<std_msgs::String>(TopicInfoPub, 1);
    activation_sub_ = nh.subscribe<std_msgs::Int32>("/tayvision/object/activation", 1, &BbxObjectDetector::callback_activation, this);
    
    activation_ = false;
    object_diff_ang_ = 100;
    object_restart_ = false;
    activation_ = false;

    object_name_ = "NULL";

    object_list[0] = "bottle";
    object_list[1] = "sports ball";
    //object_list[2] = "cell phone";
    object_list[2] = "book";
  }

void 
BbxObjectDetector::callback_activation(const std_msgs::Int32::ConstPtr& activator){
  activation_ = (bool)activator->data;
  object_restart_= (bool)activator->data;
}


void BbxObjectDetector::callback_bbx(const sensor_msgs::ImageConstPtr& image,
  const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{

  if(activation_){  
    cv_bridge::CvImagePtr img_ptr_depth;

    try
    {
        img_ptr_depth = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }

    for (const auto & box : boxes->bounding_boxes)
    {

      current_time_ = ros::Time::now().toSec();

      int px = (box.xmax + box.xmin) / 2;
      int py = (box.ymax + box.ymin) / 2;

      person_dist_ = 1;
      float dist = 1;
      //float dist = img_ptr_depth->image.at<float>(cv::Point(px, py)) * 0.001f; // * 0.001f
      float ang = -(px - CAMERA_XCENTER_) / CAMERA_XCENTER_;

      if(box.Class == "person"){
        last_time_ = ros::Time::now().toSec();
        person_ang_ = ang;
        person_dist_ = dist;
      }

      if(abs(current_time_ - last_time_) < RESTART_TIME && is_searched_object(box)){
        if(abs(ang - person_ang_) < object_diff_ang_ && abs(dist - person_dist_) < MAX_DISTANCE_DIFFERENCE_){
          object_diff_ang_ = abs(ang - person_ang_);
          object_name_ = box.Class;
        }
      }

      if(abs(current_time_ - last_time_) > RESTART_TIME || object_restart_){
        object_diff_ang_ = 100;
        person_ang_ = 10;
        object_name_ = "NULL";
      }
    }
    if(object_name_ != "NULL"){
      std::cout << "OBJECT: " << object_name_ << std::endl;
      std_msgs::String pub_object;
      pub_object.data = object_name_;
      object_pub_.publish(pub_object);
    }
  }
}

bool BbxObjectDetector::is_searched_object(darknet_ros_msgs::BoundingBox box){
  for(int i = 0; i < OBJECTS_NUM; i++){
    if(object_list[i] == box.Class){
      return true;
    }
  }
  return false;
}

}  // namespace bbx_object_detector
