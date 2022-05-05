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
#include "std_msgs/Int32.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

namespace bbx_color_detection
{

enum{
  COL_SEGMETS = 1,
  ROW_SEGMETS = 1,
};

typedef struct
{
  int r; 
  int g;
  int b;
}Rgb;

typedef struct
{
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
  void callback_activation(const std_msgs::Int32::ConstPtr& activator);

private:
  ros::NodeHandle nh;
  ros::Publisher color_pub_;
  ros::Subscriber activation_sub_;


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
  bool restart_;
  bool activation_;

  const int PUBLISH_THRESHOLD = 200;
  // ---------------------- Color identification ---------------------- //
  static const int NUM_COLORS = 8;
  Color colors_arr[NUM_COLORS];
  Color black;
  Color white;
  Color red;
  Color pink;
  Color light_blue;
  Color yellow;
  Color green;
  Color blue;
  
  const int BLACK = 0;
  const int WHITE = 1;
  const int RED = 2;
  const int PINK = 3;
  const int LIGHT_BLUE = 4;
  const int YELLOW = 5;
  const int GREEN = 6; 
  const int BLUE = 7;
  // ---------------------- ------------------- ---------------------- //

 // ---------------------- Image Debug ---------------------- //
  ros::NodeHandle image_nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  // ---------------------- ----------- ---------------------- //

  // ---------------------- Border Aliasing ---------------------- //
  const float X_PX_FACTOR  = 30; // The % of drecrease in function of init bbx
  const float Y_PX_FACTOR  = 30;
  const bool ALIASING = true;
  // ---------------------- --------------- ---------------------- //
  
  std::string detectedObject_;
  std::string TopicID;
  std::string workingFrameId_;
  std::string TopicColorPub;

  void rgb_from_bbx(darknet_ros_msgs::BoundingBox box, BbxSize bbx_size, cv::Mat src_hsv, Rgb result_hsv[COL_SEGMETS][ROW_SEGMETS], bool debug);
  int detect_color(Rgb rgb_data);
  BbxSize box_borders_aliasing(darknet_ros_msgs::BoundingBox box);
  void counters_restart();
  Color get_max_color();

};
}  // namespace bbx_color_detector

#endif  
