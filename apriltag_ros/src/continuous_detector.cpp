/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/continuous_detector.h"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(apriltag_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltag_ros
{
void ContinuousDetector::onInit ()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& pnh = getPrivateNodeHandle();


  // const cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 397.30,    0.0, 320.13,
  //                                                     0.0, 394.03, 262.50,
  //                                                     0.0,    0.0,    1.0 );//
  // // [k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,taux,tauy] of 4, 5, 8, 12 or 14 elements.
  // // If the vector is empty, the zero distortion coefficients are assumed.
  // const cv::Mat D = ( cv::Mat_<double> ( 4,1 ) << -3.5897074026113568e-01, 9.6480686476447072e-02, -2.0615787184859718e-03, -6.1006160991551920e-04);
  // const int ImgWidth = 1280;
  // const int ImgHeight = 1024;
  // cv::Mat map1, map2;
  cv::Size imageSize(ImgWidth, ImgHeight);
  const double alpha = 1;//1=保留所有像素，0=去除黑边
  cv::Mat NewCameraMatrix = getOptimalNewCameraMatrix(K, D, imageSize, alpha, imageSize, 0);
  initUndistortRectifyMap(K, D, cv::Mat(), NewCameraMatrix, imageSize, CV_32FC1, map1, map2);//CV_32FC1 CV_16SC2



  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, "publish_tag_detections_image", false);
  it_ = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh));

  std::string transport_hint;
  pnh.param<std::string>("transport_hint", transport_hint, "raw");

  // camera_image_subscriber_ = it_->subscribeCamera("image_rect", 1, &ContinuousDetector::imageCallback, this, image_transport::TransportHints(transport_hint));
  camera_image_subscriber_ = it_->subscribe("image_rect", 1, &ContinuousDetector::imageCallback, this, image_transport::TransportHints(transport_hint));

  tag_detections_publisher_ = nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }

  odomtry_publisher_ = nh.advertise<nav_msgs::Odometry>("tag_Odometry", 1);
  path_pubilsher = nh.advertise<nav_msgs::Path>("path", 1);

}

//void ContinuousDetector::imageCallback (const sensor_msgs::ImageConstPtr& image_rect, const sensor_msgs::CameraInfoConstPtr&camera_info)
void ContinuousDetector::imageCallback (const sensor_msgs::ImageConstPtr& image_rect)
{
  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_publisher_.getNumSubscribers() == 0 &&
      tag_detections_image_publisher_.getNumSubscribers() == 0 &&
      !tag_detector_->get_publish_tf())
  {
    // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
    return;
  }

  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the iamge
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  //  tag_detections_publisher_.publish(tag_detector_->detectTags(cv_image_,camera_info));
  //AprilTagDetectionArray tag_detection_array(tag_detector_->detectTags(cv_image_,camera_info));
  AprilTagDetectionArray tag_detection_array(tag_detector_->detectTags(cv_image_));

  tag_detections_publisher_.publish(tag_detection_array);

  if (false) {
    for (unsigned int i = 0; i < tag_detection_array.detections.size(); i++) {
      nav_msgs::Odometry odometry;
      odometry.header = tag_detection_array.detections[i].pose.header;
      //odometry.header.frame_id = "my_bundle";
      odometry.pose.pose = tag_detection_array.detections[i].pose.pose.pose;
      odomtry_publisher_.publish(odometry);

      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header = odometry.header;
      //pose_stamped.header.frame_id = "world";
      pose_stamped.pose.position.x = tag_detection_array.detections[i].pose.pose.pose.position.x;
      pose_stamped.pose.position.y = tag_detection_array.detections[i].pose.pose.pose.position.y;
      pose_stamped.pose.position.z = tag_detection_array.detections[i].pose.pose.pose.position.z;
      camera_path.header = pose_stamped.header;
      //camera_path.header.frame_id = "world";
      camera_path.poses.push_back(pose_stamped);
      path_pubilsher.publish(camera_path);

    }
  }

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

} // namespace apriltag_ros
