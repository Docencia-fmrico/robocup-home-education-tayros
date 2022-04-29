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
#include <image_geometry/pinhole_camera_model.h>

namespace tayfinder
{

PersonLocalizator::PersonLocalizator():
  workingFrameId_("/base_footprint"),
  detectedObject_("person"),
  image_sub(nh, "/camera/depth/image_raw", 1),
  bbx_sub(nh, "/darknet_ros/bounding_boxes", 1),
  sync_bbx(MySyncPolicy_bbx(10), image_sub, bbx_sub)
  {
    sync_bbx.registerCallback(boost::bind(&PersonLocalizator::callback_bbx, this, _1, _2));
    cam_info_taked = false;
    restart_person_status();
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
        
    if (box.Class == detectedObject_ )
    {
        int px = (box.xmax + box.xmin) / 2;
        int py = (box.ymax + box.ymin) / 2;
        float dist = img_depth_ptr->image.at<float>(cv::Point(px, py))* 0.001f;

        cv::Point2d pixel_point(px, py);
        cv::Point3d xyz = cam_model_.projectPixelTo3dRay(pixel_point);
        std::cout << "point (3D) = " << xyz << dist << std::endl << std::endl;

        PersonPoint3D current_point;
        current_point.x = dist;
        current_point.y = -xyz.x;
        current_point.z = 0;

        // VER PRIMERO SI LA PERSONA ESTA EN EL AREA
        if(!is_saved(current_point)){

        }


    }

  }

}

void PersonLocalizator::callback_cam_info(const sensor_msgs::CameraInfoConstPtr& cam_info){

    if(!cam_info_taked){
        cam_model_.fromCameraInfo(cam_info);
        cam_info_taked = true;
    }
}

void PersonLocalizator::restart_person_status(){

    for(int i = 0; i < PERSON_BUFFER; i++){
        person_list[i].status = EMPTY;
    }
    return;
}

bool PersonLocalizator::is_saved(PersonPoint3D point){
    
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


}  // namespace tayfinder

