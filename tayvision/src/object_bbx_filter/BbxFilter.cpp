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
#include "object_bbx_filter/BbxFilter.h"
#include <unistd.h>
#include <string.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace object_bbx_filter
{

BbxFilter::BbxFilter():
  it_(nh_),
  TopicPubID("/bbx_filtered/bounding_boxes/person"),
  filterobject_("person"),
  image_sub(nh_, "/camera/rgb/image_raw", 1),
  depth_image_sub(nh_, "/camera/depth/image_raw", 1),
  bbx_sub(nh_, "/darknet_ros/bounding_boxes", 1),
  sync_bbx(MySyncPolicy_bbx(10), image_sub, bbx_sub, depth_image_sub)
  {
    sync_bbx.registerCallback(boost::bind(&BbxFilter::callback_bbx, this, _1, _2, _3));
    bbx_filtered_pub_ = nh_.advertise<darknet_ros_msgs::BoundingBoxes>(TopicPubID, 1);

    first_time = true;
    last_distance_ = -1;
    move_restart_counter = 0;
    restarting = false;

    hsv_restart_counter = 0;
    publish_threshold_ = PUBLISH_TRESHOLD;
    threshold_mod_step = 1;

    image_pub_ = it_.advertise("/object_filtered_debug/image/person", 1);
    cv::namedWindow("Imagen filtrada");

  }

void BbxFilter::callback_bbx(const sensor_msgs::ImageConstPtr& image,
  const darknet_ros_msgs::BoundingBoxesConstPtr& boxes, const sensor_msgs::ImageConstPtr& depthimage)
{
  cv_bridge::CvImagePtr img_ptr;
  cv_bridge::CvImagePtr img_ptr_depth;
  cv_bridge::CvImagePtr cv_imageout;

  try
  {
    img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    cv_imageout = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    img_ptr_depth = cv_bridge::toCvCopy(depthimage, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

	cv::Mat hsv;
  hsv = img_ptr->image;
  //cv::cvtColor(img_ptr->image, hsv, CV_RGB2HSV);
	int step = hsv.step;
  int channels = 3;

  for (const auto & box : boxes->bounding_boxes)
  {

    if(box.Class == filterobject_){

      //float dist = 1; // PROBAR CON CAMARA 3D
      //float ang = 0;
      float dist = img_ptr_depth->image.at<float>(cv::Point((box.xmax + box.xmin) / 2, (box.ymax + box.ymin) / 2)) * 0.001f; // * 0.001f
      float ang = -((box.xmax + box.xmin) / 2 - CAMERA_XCENTER_) / CAMERA_XCENTER_;

      std::cout << "Dist: " << dist << "Ang: " << ang << std::endl;

      // OJO REVISAR
      if (last_distance_ < 0 || isnan(last_distance_) || isnan(last_angle_) || move_restart_counter > RESTART_TICKS_)
      {
        ROS_INFO("%s\n", "SEARCHING TARGET(MISSED)");
        restarting = true;
      }

      if ((abs(dist - last_distance_) < MAX_DISTANCE_DIFFERENCE_ && abs(ang - last_angle_) < MAX_ANGLE_DIFFERENCE_) || restarting)
      {

        // Movement filter
        if(!restarting){
          last_distance_ = dist;
          last_angle_ = ang;
        }

        // Hsv filter
        Hsv current_box_values[COL_SEGMETS][ROW_SEGMETS];
        hsv_from_bbx(box, hsv,  current_box_values, DEBUG);

        // Save te initial values of the object to filter
        if(first_time){
          for(int i = 0; i < OBJECT_HSV_LENGHT; i++){
            memcpy(&Hsv_values[i].values, &current_box_values, COL_SEGMETS*ROW_SEGMETS*sizeof(Hsv));
            Hsv_values[i].similarity = 1;

          }
          first_time = false;
        }

        // Calculate % of hsv similaritys and updete the buffer values except the [0](Initial)
        int min = 1; 
        int max = 0;
        for(int i = 0; i < OBJECT_HSV_LENGHT; i++){
          float current_similarity = calc_similarity_value(current_box_values, Hsv_values[i].values);
          Hsv_values[i].similarity = current_similarity;
          if(Hsv_values[i].similarity <= Hsv_values[min].similarity && i!= 0){
            min = i; 
          }
          if(Hsv_values[i].similarity >= Hsv_values[max].similarity){
            max = i;
          }
        }

        // Save the higher simil value
        float result_similarity = Hsv_values[max].similarity;

        if(DEBUG){
          std::cout << "DIST DIFF: " << abs(dist - last_distance_) << std::endl;
          std::cout << "DIFF ANGLE: " << abs(ang - last_angle_) << std::endl;
          std::cout << "L Dist: " << last_distance_ << std::endl;
          std::cout << "L ANG: " << last_angle_ << std::endl;
          std::cout << "Restarting: " << restarting << std::endl;
          std::cout << "MOVE COUNT: " << move_restart_counter << std::endl;
          std::cout << "HSV Counter: " << hsv_restart_counter << std::endl;
          std::cout << "Publis  thres: " << publish_threshold_ << std::endl;
          for(int i = 0; i < OBJECT_HSV_LENGHT; i++){
            std::cout << "SIMIL" << i << ": "<< Hsv_values[i].similarity << std::endl; 
          }
          std::cout << "SIMILARITY(%): " << result_similarity;
          std::cout << std::endl << std::endl;
        }

          
        if(result_similarity >= publish_threshold_)
        {
          // Restart the movement filter values
          if(restarting)
          {
            last_distance_ = dist;
            last_angle_ = ang;
            move_restart_counter = 0;
            restarting = false;
          }

          // Publish the filtered bbx
          std::cout << "Detected" << std::endl;
          darknet_ros_msgs::BoundingBoxes filtered_bbx;
          filtered_bbx.header.stamp = ros::Time::now();
          filtered_bbx.header.frame_id = "detection";
          filtered_bbx.image_header = image->header;
          filtered_bbx.bounding_boxes.push_back(box);
          bbx_filtered_pub_.publish(filtered_bbx);

          // Update 
          hsv_restart_counter = 0;
          move_restart_counter = 0;
          publish_threshold_ = PUBLISH_TRESHOLD;
          memcpy(&Hsv_values[min].values, &current_box_values, COL_SEGMETS*ROW_SEGMETS*sizeof(Hsv));

          // Image show
          if(IMAGE_DEBUG){
            
            int step = hsv.step;

            int px_width = box.xmax - box.xmin;
            int py_height = box.ymax - box.ymin;

            int px_width_segment = px_width / COL_SEGMETS;
            int py_height_segment = py_height / ROW_SEGMETS;

            int current_px =  box.xmin;
            int current_py =  box.ymin;


            //std::cout << "INITcurrent_px: " << current_px << " INITcurrent_py: " << current_py << std::endl; 

            for(int col = 0; col < COL_SEGMETS; col++){
              for(int row = 0; row < ROW_SEGMETS; row++){
               
                for(int i = current_px; i < (current_px + px_width_segment); i++){
                  for(int j = current_py; j < (current_py + py_height_segment) ; j++){
                    int position = i * CHANELS + j * step;              
                    cv_imageout->image.data[position] = current_box_values[col][row].h;
                    cv_imageout->image.data[position+1] = current_box_values[col][row].s;
                    cv_imageout->image.data[position+2] = current_box_values[col][row].v;
                  }
                }
                current_py += py_height_segment;
              }
              current_px += px_width_segment;
              current_py = box.ymin;
            }
          }
        }
        else
        {
          hsv_restart_counter++;
        }
      }
      else if(box.Class == filterobject_ && (abs(dist - last_distance_) < MAX_DISTANCE_DIFFERENCE_ || abs(ang - last_angle_) < MAX_ANGLE_DIFFERENCE_))
      {
        move_restart_counter++;
      }  
      if(hsv_restart_counter >= RESTART_HSV_TICKS)
      {
        //publish_threshold_ -= (exp(-threshold_mod_step)/9) + 0.01;
        //threshold_mod_step += 0.1;
        publish_threshold_ -= 0.001;
      }
    }
  }
  if(IMAGE_DEBUG){
    cv::imshow("Imagen filtrada", cv_imageout->image);
    cv::waitKey(3);
    image_pub_.publish(cv_imageout->toImageMsg());
  }
}

void BbxFilter::hsv_from_bbx(darknet_ros_msgs::BoundingBox box, cv::Mat src_hsv, Hsv result_hsv[COL_SEGMETS][ROW_SEGMETS], bool debug){

  int step = src_hsv.step;
  int segment_Haverage = 0; 
  int segment_Saverage = 0; 
  int segment_Vaverage = 0; 
  int segment_pixs = 0;

  int px_width = box.xmax - box.xmin;
  int py_height = box.ymax - box.ymin;

  int px_width_segment = px_width / COL_SEGMETS;
  int py_height_segment = py_height / ROW_SEGMETS;

  int current_px =  box.xmin;
  int current_py =  box.ymin;


  for(int col = 0; col < COL_SEGMETS; col++){
    for(int row = 0; row < ROW_SEGMETS; row++){
      segment_Haverage = 0; 
      segment_Saverage = 0; 
      segment_Vaverage = 0; 
      segment_pixs = 0;

      for(int i = current_px; i < (current_px + px_width_segment); i++){
        for(int j = current_py; j < (current_py + py_height_segment) ; j++){
          int position = i * CHANELS + j * step;              
          segment_Haverage += src_hsv.data[position];
          segment_Saverage += src_hsv.data[position + 1];
          segment_Vaverage += src_hsv.data[position + 2];
          segment_pixs++;
        }
      }
      current_py += py_height_segment;
      result_hsv[col][row].h = segment_Haverage / segment_pixs;
      result_hsv[col][row].s = segment_Saverage / segment_pixs;
      result_hsv[col][row].v = segment_Vaverage / segment_pixs;
    }
    current_px += px_width_segment;
    current_py = box.ymin;
  }

  if(debug){
  for(int col = 0; col < COL_SEGMETS; col++){
    for(int row = 0; row < ROW_SEGMETS; row++){
      std::cout << " ["<< result_hsv[col][row].h << " " << result_hsv[col][row].s << " " << result_hsv[col][row].v << "]";
    }
    std::cout << std::endl;
  }
    std::cout << std::endl;
  }
}


float BbxFilter::calc_similarity_value(Hsv current_hsv_values[COL_SEGMETS][ROW_SEGMETS], Hsv hsv_to_compare[COL_SEGMETS][ROW_SEGMETS]){

  float simil_value = 0;
  float current_Hdiff = 0;
  float current_Sdiff = 0;
  float current_Vdiff = 0;
  
  for(int col = 0; col < COL_SEGMETS; col++){
    for(int row = 0; row < ROW_SEGMETS; row++){
      current_Hdiff = abs(current_hsv_values[col][row].h - hsv_to_compare[col][row].h);
      current_Sdiff = abs(current_hsv_values[col][row].s - hsv_to_compare[col][row].s);
      current_Vdiff = abs(current_hsv_values[col][row].v - hsv_to_compare[col][row].v);
      simil_value += (((-current_Hdiff/DIFF_TRESHOLD) + 1) * MAX_PERCENT_FOR_SEGMET) * H_VALUE_IMPORTANCE;
      simil_value += (((-current_Sdiff/DIFF_TRESHOLD) + 1) * MAX_PERCENT_FOR_SEGMET) * S_VALUE_IMPORTANCE;
      simil_value += (((-current_Vdiff/DIFF_TRESHOLD) + 1) * MAX_PERCENT_FOR_SEGMET) * V_VALUE_IMPORTANCE;
    }
  }
  
  return simil_value;
}

}  // namespace object_bbx_filter

