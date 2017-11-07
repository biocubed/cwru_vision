/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Case Western Reserve University
 *    Russell Jackson <rcj33@case.edu>
 *    Tipakorng Greigarn <txg92@case.edu>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve Univeristy, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CWRU_OPENCV_COMMON_MANUAL_LABELING_H
#define CWRU_OPENCV_COMMON_MANUAL_LABELING_H

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <cwru_opencv_common/opencv_local.h>
#include <cwru_opencv_common/opencv_ui.h>
#include <cv_bridge/cv_bridge.h>
#include <cwru_opencv_common/image_label.h>
#include <image_transport/image_transport.h>

/**
 * \brief This class is a manual labeling server.
 */
class ManualLabeling
{
public:
  /**
   * \brief Constructor
   */
  explicit ManualLabeling(ros::NodeHandle &nodeHandle);

  /**
   * \brief Destructor
   */
  ~ManualLabeling();

  /**
   * \brief Manual labeling callback function
   */
  bool manualLabelingCallback(cwru_opencv_common::image_label::Request& request,
    cwru_opencv_common::image_label::Response& response);

  /**
   * \brief image subscription callback.
   */
  void newImageCallback(const sensor_msgs::ImageConstPtr& msg);

protected:
  /**
   * \brief ROS node handle
   */
  ros::NodeHandle nodeHandle_;

  /**
   * \brief Manual image labeling server
   */
  ros::ServiceServer manualLabelingServer_;

  /**
   * \brief image transport object.
   */
  image_transport::ImageTransport it;

  /**
   * \brief local image def.
   */
  cv::Mat localImage;


  /**
   * \brief image point
   */
  cv::Point imagePt;

  /**
   * \brief is an image ready
   */
  bool imageRdy;

  image_transport::Subscriber img_sub;

  geometry_msgs::Polygon ptList;
};

#endif  // CWRU_OPENCV_COMMON_MANUAL_LABELING_H
