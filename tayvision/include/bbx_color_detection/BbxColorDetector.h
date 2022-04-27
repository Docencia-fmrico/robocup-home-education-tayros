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

#ifndef BBX_COLOR_DETECTOR_BBX_COLOR_DETECTOR
#define BBX_COLOR_DETECTOR_BBX_COLOR_DETECTOR

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

namespace bbx_color_detection
{

enum{
  COL_SEGMETS = 10,
  ROW_SEGMETS = 10,
};

typedef struct
{
  int r; 
  int g;
  int b;
}Rgb;

typedef struct
{
  int rupper; 
  int rlower;
  int gupper; 
  int glower;
  int bupper; 
  int blower;
  int counter;
  std::string name;
}Color;

typedef struct
{
  int x_min;
  int x_max;
  int y_min; 
  int y_max;
}BbxSize;

class BbxColorDetector
{
public:
  BbxColorDetector();
  void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);
  void rgb_from_bbx(darknet_ros_msgs::BoundingBox box, BbxSize bbx_size, cv::Mat src_hsv, Rgb result_hsv[COL_SEGMETS][ROW_SEGMETS], bool debug);
  bool check_color(Rgb rgb_data , Color color);
  BbxSize box_borders_aliasing(darknet_ros_msgs::BoundingBox box);

private:
  ros::NodeHandle nh;

  const bool DEBUG = true;
  const bool IMAGE_DEBUG = true;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
  darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
  message_filters::Subscriber<sensor_msgs::Image> image_sub;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;

  const float CAMERA_XCENTER_ = 340;
  const float MAX_DISTANCE_DIFFERENCE_ = 0.2;
  const float MAX_ANGLE_DIFFERENCE_ = 0.1;
  const int RESTART_TICKS_ = 100;
  static const int CHANNELS = 3;

  float last_distance_;  // Save last distance measured
  float last_angle_;
  int restart_counter;
  bool bbx_restart;
  
  // ---------------------- Color identification ---------------------- //
  static const int NUM_COLORS = 8;
  Color colors_arr[NUM_COLORS];
  Color black;
  Color red;
  Color green;
  Color blue;
  Color yellow;
  Color magenta;
  Color white;
  Color orange;
  Color whithe;
  // ---------------------- ------------------- ---------------------- //

 // ---------------------- Image Debug ---------------------- //
  ros::NodeHandle image_nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  // ---------------------- ----------- ---------------------- //

  // ---------------------- Border Aliasing ---------------------- //
  const int COL_ALIASING_FACTOR_TOP = 10; /* bbx crop part, if 10 1/10 of the image croped */
  const int COL_ALIASING_FACTOR_BOT = 10;
  const int ROW_ALIASING_FACTOR_TOP = 5;
  const int ROW_ALIASING_FACTOR_BOT = 100;
  const bool ALIASING = true;
  // ---------------------- --------------- ---------------------- //
  
  std::string detectedObject_;
  std::string TopicID;
  std::string workingFrameId_;
};
}  // namespace bbx_color_detector

#endif  
