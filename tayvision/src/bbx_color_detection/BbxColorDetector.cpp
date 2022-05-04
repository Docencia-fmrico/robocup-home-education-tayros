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
#include "bbx_color_detection/BbxColorDetector.h"
#include <unistd.h>
#include <string.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace bbx_color_detection
{

BbxColorDetector::BbxColorDetector():
	it_(nh),
  workingFrameId_("/base_footprint"),
  detectedObject_("person"),
  TopicColorPub("tayvision/person/color"),
  image_sub(nh, "/usb_cam/image_raw", 1),
  bbx_sub(nh, "/darknet_ros/bounding_boxes", 1),
  sync_bbx(MySyncPolicy_bbx(10), image_sub, bbx_sub)
  {
    sync_bbx.registerCallback(boost::bind(&BbxColorDetector::callback_bbx, this, _1, _2));
    color_pub_ = nh.advertise<std_msgs::String>(TopicColorPub, 1);

		image_pub_ = it_.advertise("/object_filtered_debug/image/person", 1);
    cv::namedWindow("Imagen filtrada");

    black.name = "BLACK";
    colors_arr[0] = black;

    white.name = "WHITE";
    colors_arr[1] = white;

    red.name = "RED";
    colors_arr[2] = red;

    pink.name = "PINK";
    colors_arr[3] = pink;

    light_blue.name = "LIGHT_BLUE";
    colors_arr[4] = light_blue;

    yellow.name = "YELLOW";
    colors_arr[5] = yellow;

    green.name = "GREEN";
    colors_arr[6] = green;

    blue.name = "BLUE";
    colors_arr[7] = blue;

    counters_restart();
    restart_ = true;
    }

void BbxColorDetector::callback_bbx(const sensor_msgs::ImageConstPtr& image,
  const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
  cv_bridge::CvImagePtr img_ptr;
  cv_bridge::CvImagePtr cv_imageout;

  try
  {
    img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    cv_imageout = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
  }

  cv::Mat rgb;
	rgb = img_ptr->image;
	int step = rgb.step;

  if(restart_){
    restart_ = false;
    counters_restart();
  }

	for (const auto & box : boxes->bounding_boxes)
  {
    
    BbxSize bbx_aliased;
    bbx_aliased = box_borders_aliasing(box);

    Rgb  rgb_current_values[COL_SEGMETS][ROW_SEGMETS];
    rgb_from_bbx(box, bbx_aliased, rgb, rgb_current_values, DEBUG);

    // Color study 
    for(int col = 0; col < COL_SEGMETS; col++){
				for(int row = 0; row < ROW_SEGMETS; row++){
          detect_color(rgb_current_values[col][row]);
        }
      }


    // Results
    for (int i = 0; i < NUM_COLORS; i++){
      std::cout <<  "Color: "<<  colors_arr[i].name << ":\t" << colors_arr[i].counter << std::endl;
    }

    Color max_color;
    max_color = get_max_color();

    if(max_color.counter > PUBLISH_THRESHOLD){
      std_msgs::String pub_color;
      pub_color.data = max_color.name;
      color_pub_.publish(pub_color);
    }

		if(IMAGE_DEBUG){
			
			int step = rgb.step;

      int px_width = bbx_aliased.x_max - bbx_aliased.x_min;
      int py_height = bbx_aliased.y_max - bbx_aliased.y_min;

      int px_width_segment = px_width / COL_SEGMETS;
      int py_height_segment = py_height / ROW_SEGMETS;

      int current_px =  bbx_aliased.x_min;
      int current_py =  bbx_aliased.y_min;


			//std::cout << "INITcurrent_px: " << current_px << " INITcurrent_py: " << current_py << std::endl; 

			for(int col = 0; col < COL_SEGMETS; col++){
				for(int row = 0; row < ROW_SEGMETS; row++){
					
					for(int i = current_px; i < (current_px + px_width_segment); i++){
						for(int j = current_py; j < (current_py + py_height_segment) ; j++){
							int position = i * CHANNELS + j * step;              
							cv_imageout->image.data[position] = rgb_current_values[col][row].b;
							cv_imageout->image.data[position+1] = rgb_current_values[col][row].g;
							cv_imageout->image.data[position+2] = rgb_current_values[col][row].r;
						}
					}
					current_py += py_height_segment;
				}
				current_px += px_width_segment;
				current_py = bbx_aliased.y_min;
			}
		}
	}

	if(IMAGE_DEBUG){
    cv::imshow("Imagen filtrada", cv_imageout->image);
    cv::waitKey(3);
    image_pub_.publish(cv_imageout->toImageMsg());
  }

}

void BbxColorDetector::rgb_from_bbx(darknet_ros_msgs::BoundingBox box, BbxSize bbx_size, cv::Mat src_rgb, Rgb result_hsv[COL_SEGMETS][ROW_SEGMETS], bool debug){

  int step = src_rgb.step;
  int segment_Baverage = 0; 
  int segment_Gaverage = 0; 
  int segment_Raverage = 0; 
  int segment_pixs = 0;

  int px_width = bbx_size.x_max - bbx_size.x_min;
  int py_height = bbx_size.y_max - bbx_size.y_min;

  int px_width_segment = px_width / COL_SEGMETS;
  int py_height_segment = py_height / ROW_SEGMETS;

  int current_px =  bbx_size.x_min;
  int current_py =  bbx_size.y_min;


  for(int col = 0; col < COL_SEGMETS; col++){
    for(int row = 0; row < ROW_SEGMETS; row++){
      segment_Baverage = 0; 
      segment_Gaverage = 0; 
      segment_Raverage = 0; 
      segment_pixs = 0;

      for(int i = current_px; i < (current_px + px_width_segment); i++){
        for(int j = current_py; j < (current_py + py_height_segment) ; j++){
          int position = i * CHANNELS + j * step;              
          segment_Baverage += src_rgb.data[position];
          segment_Gaverage += src_rgb.data[position + 1];
          segment_Raverage += src_rgb.data[position + 2];
          segment_pixs++;
        }
      }
      current_py += py_height_segment;
      result_hsv[col][row].b = segment_Baverage / segment_pixs;
      result_hsv[col][row].g = segment_Gaverage / segment_pixs;
      result_hsv[col][row].r = segment_Raverage / segment_pixs;
    }
    current_px += px_width_segment;
    current_py = bbx_size.y_min;
  }

  if(debug){
  for(int col = 0; col < COL_SEGMETS; col++){
    for(int row = 0; row < ROW_SEGMETS; row++){
      std::cout << " ["<< result_hsv[col][row].r << " " << result_hsv[col][row].g << " " << result_hsv[col][row].b << "]";
    }
    std::cout << std::endl;
  }
    std::cout << std::endl;
  }
}

int BbxColorDetector::detect_color(Rgb rgb_data){

  int r = rgb_data.r;
  int g = rgb_data.g;
  int b = rgb_data.b;

  if(r < 100 && g < 100 && b <= 100){
    colors_arr[BLACK].counter++;
    return BLACK;
  }
  if(r >= 150 && b >= 200 && g >= 150){
    colors_arr[WHITE].counter++;
    return WHITE;
  }

  if(r > g && r > b){
    if(b <= g + 60 && b >= g - 60){
      colors_arr[RED].counter++;
      return RED;
    }
    else if(b > g + 60){
      colors_arr[PINK].counter++;
      return PINK;
    }
    else if(g > b + 60){
      colors_arr[YELLOW].counter++;
      return YELLOW;
    }
  }
  
  if(g > r && g >= b){
    if(r <= b + 60 && r >= b - 60){
      colors_arr[GREEN].counter++;
      return GREEN;
    }
    else if(r > b + 60){
      colors_arr[YELLOW].counter++;
      return YELLOW;
    }
    else if(b > r + 60){
      colors_arr[LIGHT_BLUE].counter++;
      return LIGHT_BLUE;
    }
  }

  if(b > r && b > g){
    if(g <= r + 60 && g >= r - 60){
      colors_arr[BLUE].counter++;
      return BLUE;
    }
    else if(g > r + 60){
      colors_arr[LIGHT_BLUE].counter++;
      return LIGHT_BLUE;
    }
    else if(r > g + 60){
      colors_arr[PINK].counter++;
      return PINK;
    }
  }
  
  return -1;
  
}

void BbxColorDetector::counters_restart(){
  for(int i = 0; i <NUM_COLORS; i++){
    colors_arr[i].counter = 0;
  }
}

Color BbxColorDetector::get_max_color(){
  int max =  colors_arr[0].counter;
  int max_pos = 0;

  for(int i = 0; i <NUM_COLORS; i++){
    if(colors_arr[i].counter > max){
      max = colors_arr[i].counter;
      max_pos = i;
    }
  }
  return colors_arr[max_pos];
}


BbxSize BbxColorDetector::box_borders_aliasing(darknet_ros_msgs::BoundingBox box){

  BbxSize bbx_aliased;

  if(ALIASING){

    int x_wide = box.xmax - box.xmin;
    int y_high = box.ymax - box.ymin;

    int x_center = box.xmax - (x_wide / 2);
    int y_center = box.ymax - (y_high / 2);

    bbx_aliased.x_max = x_center + (x_wide / X_PX_FACTOR * 2);
    bbx_aliased.x_min = x_center - (x_wide / X_PX_FACTOR * 2);
    bbx_aliased.y_max = y_center + (y_high / Y_PX_FACTOR * 2);
    bbx_aliased.y_min = y_center - (y_high / Y_PX_FACTOR * 2);

    std::cout << "Center X: " << x_center <<  std::endl; 
    std::cout << "Center Y: " << y_center <<  std::endl;
    std::cout << "PX MIN: " << box.xmin << " Aliased: " << bbx_aliased.x_min << std::endl; 
    std::cout << "PX MAX: " << box.xmax << " Aliased: " << bbx_aliased.x_max << std::endl; 
    std::cout << "PY MAX: " << box.ymax << " Aliased: " << bbx_aliased.y_max << std::endl; 
    std::cout << "PY MIN: " << box.ymin << " Aliased: " << bbx_aliased.y_min << std::endl; 
  }
  else{
    bbx_aliased.x_max = box.xmax;
    bbx_aliased.x_min = box.xmin;
    bbx_aliased.y_max = box.ymax;
    bbx_aliased.y_min = box.ymin;
  }
  
  return bbx_aliased;
}


}  // namespace bbx_color_detection

