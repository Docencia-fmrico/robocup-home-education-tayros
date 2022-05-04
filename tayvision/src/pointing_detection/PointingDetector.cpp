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
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "pointing_detection/PointingDetector.h"


namespace tayvision
{

PointingDetector::PointingDetector()
: nh(),
  workingFrameId_("/base_footprint"),
  detectedObject_("person"),
  image_depth_sub(nh, "/camera/rgb/image_raw", 1),
  bbx_sub(nh, "/darknet_ros/bounding_boxes", 1),
  sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub)
  {
    sync_bbx.registerCallback(boost::bind(&PointingDetector::callback_bbx, this, _1, _2));
    last_distance_ = -1;
    restart_counter = 0;
    pointing_activation = false;
    pointing_ticks_counter = 0;
    bbx_restart = false;

    std::string act_sub = nh.param("activation_topic", std::string("/tayros/pointer_detector_activation"));
    std::string Bbx_sub = nh.param("bbx_reset_topic", std::string("/tayros/bbx_reset"));
    std::string side_pub = nh.param("suticase_side_topic", std::string("/tayros/suitcase_side"));
    
    
    suitcase_side_ = nh.advertise<std_msgs::Int32>(side_pub, 1);
    activation_sub_ =  nh.subscribe<std_msgs::Int32>(act_sub, 1, &PointingDetector::activation_callback, this);
    bbx_restart_sub_ = nh.subscribe<std_msgs::Int32>(Bbx_sub, 1, &PointingDetector::bbx_restart_callback, this);
  }

void PointingDetector::callback_bbx(const sensor_msgs::ImageConstPtr& image,
  const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{

  if(pointing_activation){
    std_msgs::Int32 state;
    cv_bridge::CvImagePtr img_ptr_depth;

    try
    {
        //img_ptr_depth = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }

    for (const auto & box : boxes->bounding_boxes)
    {
      
      int px = (box.xmax + box.xmin) / 2;
      int py = (box.ymax + box.ymin) / 2;

      float dist = 1;
      float ang = 0;
      //float dist = img_ptr_depth->image.at<float>(cv::Point(px, py)) * 0.001f; // * 0.001f
      //float ang = -(px - CAMERA_XCENTER_) / CAMERA_XCENTER_;


      if (last_distance_ < 0 || isnan(last_distance_) || isnan(last_angle_) || restart_counter > RESTART_TICKS_)
      {
        ROS_INFO("%s\n", "CHANGING TARGET");
        last_distance_ = dist;
        last_angle_ = ang;
        restart_counter = 0;
      }

      ROS_INFO("%s%f%s%f\n", "LAST DISTANCE: ",last_distance_, " CURRENT DIST: ", dist);
      ROS_INFO("%s%f%s%f\n", "LAST ANGLE: ",last_angle_, " CURRENT ang: ", ang);
      ROS_INFO("%s%ld%s%ld\n", "PX MIN: ",box.xmin, " PX MAX ", box.xmax);
      ROS_INFO("%s%ld%s%ld\n", "PY MIN: ",box.ymin, " PY MAX ", box.ymax);

      if (box.Class == detectedObject_ && abs(dist - last_distance_) < MAX_DISTANCE_DIFFERENCE_ && abs(ang - last_angle_) < MAX_ANGLE_DIFFERENCE_)
      {
        std::cerr << box.Class << " at (" << "Dist: "<< dist << " Ang: " <<ang << std::endl;
  
        last_distance_ = dist;
        last_angle_ = ang;
        restart_counter = 0;

        // Pointing 

        if(bbx_restart){
          last_px_min_ = box.xmin;
          last_px_max_ = box.xmax;
          bbx_restart = false; //DEBUG
        }

        if(pointing_activation == TRUE){

          int left_diff = last_px_min_ - box.xmin;
          int rigth_diff = box.xmax - last_px_max_;

          ROS_INFO("%s%d%s%d\n", "Left Diff: ", left_diff, "Right diff ", rigth_diff); 


          if((left_diff > MIN_PX_CAHNGE && rigth_diff > MIN_PX_CAHNGE)){
              ROS_INFO("%s\n", "NO POINTING"); 
              if(left_diff > rigth_diff){
                  state.data = LEFT;
                  suitcase_side_.publish(state);
                  ROS_INFO("%s\n", "POINTING LEFT"); 
              }
              else{
                  state.data = RIGHT;
                  suitcase_side_.publish(state);
                  ROS_INFO("%s\n", "POINTING RIGHT");
              }
          }

          else if(left_diff > MIN_PX_CAHNGE){
            state.data = LEFT;
            suitcase_side_.publish(state);
            ROS_INFO("%s\n", "POINTING LEFT"); 
          }

          else if(rigth_diff > MIN_PX_CAHNGE){
            state.data = RIGHT;
            suitcase_side_.publish(state);
            ROS_INFO("%s\n", "POINTING RIGHT"); 
          }
          pointing_ticks_counter++;
        }
      }
      if(box.Class == detectedObject_ && abs(dist - last_distance_) > MAX_DISTANCE_DIFFERENCE_){
        restart_counter++;
      }
    }

  }
}

void 
PointingDetector::activation_callback(const std_msgs::Int32::ConstPtr& msg)
{
  pointing_activation = msg->data;
}

void 
PointingDetector::bbx_restart_callback(const std_msgs::Int32::ConstPtr& msg)
{
  if (msg->data == TRUE)
  {
    bbx_restart = true;
  }
  else
  {
    bbx_restart = false;
  }
}



}  // namespace human_perception
