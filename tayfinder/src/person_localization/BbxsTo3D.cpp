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
#include "person_localizator/BbxsTo3D.h"
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>


namespace tayfinder
{

BbxsTo3D::BbxsTo3D()
: nh(),
  workingFrameId_("base_footprint"),
  ObjectFrameId_("bbx_person"),
  detectedObject_("person"),
  image_depth_sub(nh, "/camera/depth/image_raw", 1),
	cam_info_sub(nh, "/camera/depth/camera_info", 1),
  bbx_sub(nh, "/darknet_ros/bounding_boxes", 1),
  sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub),
	sync_cam(MySyncPolicy_cam(10), image_depth_sub, cam_info_sub)
  {
    sync_bbx.registerCallback(boost::bind(&BbxsTo3D::callback_bbx, this, _1, _2));
		sync_cam.registerCallback(boost::bind(&BbxsTo3D::callback_cam_info, this, _1, _2));
  }

void BbxsTo3D::callback_bbx(const sensor_msgs::ImageConstPtr& image,
  const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
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
 
			ROS_INFO("BOX");

    if (box.Class == detectedObject_  && cam_info_taked)
    {
			int px = (box.xmax + box.xmin) / 2;
    	int py = (box.ymax + box.ymin) / 2;
			float dist = img_ptr_depth->image.at<float>(cv::Point(px, py)); // * 0.001f

			cv::Point2d pixel_point(px, py);
			cv::Point3d xyz = cam_model_.projectPixelTo3dRay(pixel_point);
			std::cout << "point (3D) = " << xyz << dist << std::endl << std::endl;

			tf::StampedTransform transform;
			transform.setOrigin(tf::Vector3(dist, -xyz.x, 0.0));
			transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

			transform.stamp_ = ros::Time::now();
			transform.frame_id_ = workingFrameId_;
			transform.child_frame_id_ = ObjectFrameId_; 

			try
			{
				tfBroadcaster_.sendTransform(transform);
			}
			catch(tf::TransformException& ex)
			{
				ROS_ERROR_STREAM("Transform error of sensor data(BbxTo3D): " << ex.what() << ", quitting callback");
				return;
			}
		}
	}
}



void BbxsTo3D::callback_cam_info(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info){

	if(!cam_info_taked){
		ROS_INFO("CAM INIT");
		cam_model_.fromCameraInfo(cam_info);
		cam_info_taked = true;
	}
}




}  // namespace tayfinder
