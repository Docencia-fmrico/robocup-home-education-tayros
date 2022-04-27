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
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace bbx_color_detection
{

BbxColorDetector::BbxColorDetector():
	it_(nh),
  workingFrameId_("/base_footprint"),
  detectedObject_("person"),
  image_sub(nh, "/usb_cam/image_raw", 1),
  bbx_sub(nh, "/darknet_ros/bounding_boxes", 1),
  sync_bbx(MySyncPolicy_bbx(10), image_sub, bbx_sub)
  {
    sync_bbx.registerCallback(boost::bind(&BbxColorDetector::callback_bbx, this, _1, _2));

		image_pub_ = it_.advertise("/object_filtered_debug/image/person", 1);
    cv::namedWindow("Imagen filtrada");

		black.rlower = 0;
    black.rupper = 96;
    black.glower = 0;
    black.gupper = 96;
    black.blower = 0;
    black.bupper = 96;
    black.name = "BLACK";
    black.counter = 0;
    colors_arr[0] = black;

    red.rlower = 128;
    red.rupper = 255;
    red.glower = 0;
    red.gupper = 153;
    red.blower = 0;
    red.bupper = 153;
    red.name = "RED";
    red.counter = 0;
    colors_arr[1] = red;

	  orange.rlower = 128;
    orange.rupper = 255;
    orange.glower = 128;
    orange.gupper = 204;
    orange.blower = 0;
    orange.bupper = 153;
    orange.name = "ORANGE";
    orange.counter = 0;
    colors_arr[2] = orange;

    yellow.rlower = 153;
    yellow.rupper = 255;
    yellow.glower = 153;
    yellow.gupper = 255;
    yellow.blower = 0;
    yellow.bupper = 153;
    yellow.name = "YELLOW";
    yellow.counter = 0;
    colors_arr[3] = yellow;

    green.rlower = 0;
    green.rupper = 128;
    green.glower = 128;
    green.gupper = 255;
    green.blower = 0;
    green.bupper = 128;
    green.name = "GREEN";
    green.counter = 0;
    colors_arr[4] = green;

    blue.rlower = 0;
    blue.rupper = 153;
    blue.glower = 0;
    blue.gupper = 255;
    blue.blower = 128;
    blue.bupper = 255;
    blue.name = "BLUE";
    blue.counter = 0;
    colors_arr[5] = blue;

    magenta.rlower = 204;
    magenta.rupper = 0;
    magenta.glower = 0;
    magenta.gupper = 153;
    magenta.blower = 102;
    magenta.bupper = 204;
    magenta.name = "PURPLE";
    magenta.counter = 0;
    colors_arr[6] = magenta;

    whithe.rlower = 204;
    whithe.rupper = 255;
    whithe.glower = 204;
    whithe.gupper = 255;
    whithe.blower = 204;
    whithe.bupper = 255;
    whithe.name = "WHITHE";
    whithe.counter = 0;
    colors_arr[7] = whithe;

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

	cv::Mat rgb = img_ptr->image;
	int step = rgb.step;
	for (const auto & box : boxes->bounding_boxes)
  {
    // Box borders aliasing
    int b_w = box.xmax - box.xmin;
    int b_h = box.ymax - box.ymin;

    box_xmax = box.xmax - (b_w / 10);
    box_xmin = box.xmin + (b_w / 10);

    Rgb  rgb_current_values[COL_SEGMETS][ROW_SEGMETS];
    rgb_from_bbx(box, rgb,  rgb_current_values, DEBUG);

    // Color study 
    for(int col = 0; col < COL_SEGMETS; col++){
				for(int row = 0; row < ROW_SEGMETS; row++){
          
          for(int i = 0; i < NUM_COLORS; i++){
            if(check_color(rgb_current_values[col][row], colors_arr[i])){
              colors_arr[i].counter++;
              std::cout <<  colors_arr[i].name << "\t";
            }
            else if(i == NUM_COLORS - 1){
              std::cout <<  "NULL" << "\t";
            }
          }

      }
        std::cout << std::endl;
    }

    // Results
    for (int i = 0; i < NUM_COLORS; i++){
      std::cout <<  colors_arr[i].name << ":\t" << colors_arr[i].counter << std::endl;
    }



		if(IMAGE_DEBUG){
			
			int step = rgb.step;

      int px_width = box_xmax - box_xmin;
      int py_height = box.ymax - box.ymin;

      int px_width_segment = px_width / COL_SEGMETS;
      int py_height_segment = py_height / ROW_SEGMETS;

      int current_px =  box_xmin;
      int current_py =  box.ymin;


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
				current_py = box.ymin;
			}
		}
	}

	if(IMAGE_DEBUG){
    cv::imshow("Imagen filtrada", cv_imageout->image);
    cv::waitKey(3);
    image_pub_.publish(cv_imageout->toImageMsg());
  }

}

void BbxColorDetector::rgb_from_bbx(darknet_ros_msgs::BoundingBox box, cv::Mat src_rgb, Rgb result_hsv[COL_SEGMETS][ROW_SEGMETS], bool debug){

  // Px modified TAKE CARE 
  int step = src_rgb.step;
  int segment_Haverage = 0; 
  int segment_Saverage = 0; 
  int segment_Vaverage = 0; 
  int segment_pixs = 0;

  int px_width = box_xmax - box_xmin;
  int py_height = box.ymax - box.ymin;

  int px_width_segment = px_width / COL_SEGMETS;
  int py_height_segment = py_height / ROW_SEGMETS;

  int current_px =  box_xmin;
  int current_py =  box.ymin;


  for(int col = 0; col < COL_SEGMETS; col++){
    for(int row = 0; row < ROW_SEGMETS; row++){
      segment_Haverage = 0; 
      segment_Saverage = 0; 
      segment_Vaverage = 0; 
      segment_pixs = 0;

      for(int i = current_px; i < (current_px + px_width_segment); i++){
        for(int j = current_py; j < (current_py + py_height_segment) ; j++){
          int position = i * CHANNELS + j * step;              
          segment_Haverage += src_rgb.data[position];
          segment_Saverage += src_rgb.data[position + 1];
          segment_Vaverage += src_rgb.data[position + 2];
          segment_pixs++;
        }
      }
      current_py += py_height_segment;
      result_hsv[col][row].b = segment_Haverage / segment_pixs;
      result_hsv[col][row].g = segment_Saverage / segment_pixs;
      result_hsv[col][row].r = segment_Vaverage / segment_pixs;
    }
    current_px += px_width_segment;
    current_py = box.ymin;
  }

  if(debug){
  for(int col = 0; col < COL_SEGMETS; col++){
    for(int row = 0; row < ROW_SEGMETS; row++){
      std::cout << " ["<< result_hsv[col][row].b << " " << result_hsv[col][row].g << " " << result_hsv[col][row].r << "]";
    }
    std::cout << std::endl;
  }
    std::cout << std::endl;
  }
}

bool BbxColorDetector::check_color(Rgb rgb_data , Color color){

  if(rgb_data.r >= color.rlower && rgb_data.r <= color.rupper && 
    rgb_data.g >= color.glower && rgb_data.g <= color.gupper &&
    rgb_data.b >= color.blower && rgb_data.b <= color.bupper){
      return true;
  }
  return false;
}


}  // namespace bbx_color_detection

