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

#ifndef OBJECT_FILTER_BBXFILTER_H
#define OBJECT_FILTER_BBXFILTER_H

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

namespace object_bbx_filter
{

enum{
  COL_SEGMETS = 10,
  ROW_SEGMETS = 10
};

typedef struct
{
  int h; 
  int s;
  int v;
}Hsv;

typedef struct
{
  int hupper; 
  int hlower;
  int supper; 
  int slower;
  int vupper; 
  int vlower;
}Color;

typedef struct{
  Hsv values[COL_SEGMETS][ROW_SEGMETS];
  float similarity;
}Object_HSV;


class BbxFilter
{
public:
  BbxFilter();
  void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes, const sensor_msgs::ImageConstPtr& depthimage);

private:
  ros::NodeHandle nh_;
  const bool DEBUG = true;
  const bool IMAGE_DEBUG = false;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,darknet_ros_msgs::BoundingBoxes,
  sensor_msgs::Image> MySyncPolicy_bbx;
  message_filters::Subscriber<sensor_msgs::Image> image_sub;
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;

  // ---------------------------- HSV Filter --------------------------- //
  
  ros::Publisher bbx_filtered_pub_;
  static const int HSV_SEGMENTS = ROW_SEGMETS * COL_SEGMETS;
  static const int DIFF_TRESHOLD = 75;  //HSV DIFF TO TAKE MINIMUM PERCENT
  static const int CHANELS = 3;
  const float MAX_PERCENT_FOR_SEGMET = 1.0 / HSV_SEGMENTS;
  const float H_VALUE_IMPORTANCE= 0.4 ;      // The part that the Hue value gonna contributte[0 - 1]
  const float S_VALUE_IMPORTANCE= 0.4 ;      // The sum of the 3 need to be 1
  const float V_VALUE_IMPORTANCE= 0.2 ;
  const float INIT_VALUES_IMPORTANCE = 0.5;  // The part that the diff with init values gonna contributte[0 - 1]
  const float PUBLISH_TRESHOLD = 0.7;        // The minimun simil to publish data
  const int RESTART_HSV_TICKS = 100;
  static const int OBJECT_HSV_LENGHT = 5;

  bool first_time; // DEBUG
  float init_values_importance;
  int hsv_restart_counter;
  float publish_threshold_;
  float threshold_mod_step;

  Object_HSV Hsv_values[OBJECT_HSV_LENGHT];
  
  float calc_similarity_value(Hsv current_hsv_values[COL_SEGMETS][ROW_SEGMETS], Hsv hsv_to_compare[COL_SEGMETS][ROW_SEGMETS]);
  void hsv_from_bbx(darknet_ros_msgs::BoundingBox box, cv::Mat src_hsv, Hsv result_hsv[COL_SEGMETS][ROW_SEGMETS], bool debug);
  // ---------------------------- --------- ----------------------------- //

  // ------------------------- Movement Filter ------------------------- //
  const float CAMERA_XCENTER_ = 340;
  const float MAX_DISTANCE_DIFFERENCE_ = 0.4;
  const float MAX_ANGLE_DIFFERENCE_ = 0.2;
  const int RESTART_TICKS_ = 100;

  float last_distance_;  // Save last distance measured
  float last_angle_;
  int move_restart_counter;
  bool restarting;

  // ------------------------- --------------- ------------------------- //

  // ---------------------- Image Debug ---------------------- //
  ros::NodeHandle image_nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  // ---------------------- ----------- ---------------------- //
  

  std::string filterobject_;
  std::string TopicPubID;
};
}  // namespace object_bbx_filter

#endif  // OBJECT_FILTER_BBXFILTER_H
