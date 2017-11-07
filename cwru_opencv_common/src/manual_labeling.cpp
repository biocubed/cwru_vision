/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Case Western Reserve University
 *    Tipakorng Greigarn <txg92@case.edu>
 *    Russell Jackson <rcj33@case.edu>
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

#include <cwru_opencv_common/manual_labeling.h>

ManualLabeling::ManualLabeling(ros::NodeHandle &nodeHandle) :
nodeHandle_(nodeHandle), it(nodeHandle_), imagePt(-1, -1), imageRdy(false)
{
  manualLabelingServer_ = nodeHandle_.advertiseService("manual_labeling_service",
    &ManualLabeling::manualLabelingCallback, this);

  img_sub = it.subscribe("/catadioptric_play/image_raw", 1,
    boost::function < void(const sensor_msgs::ImageConstPtr &)>
    (boost::bind(&ManualLabeling::newImageCallback, this, _1)));

  cv::namedWindow("Selectable Points");
  cv::setMouseCallback("Selectable Points", cv_ui::getCoordinates, &imagePt);
  cv::startWindowThread();
  ROS_INFO("Manual image labeling  server initialized");

}

ManualLabeling::~ManualLabeling()
{
  cv::destroyWindow("Selectable Points");
}

bool ManualLabeling::manualLabelingCallback(cwru_opencv_common::image_label::Request& request,
  cwru_opencv_common::image_label::Response& response)
{
  if (!imageRdy) return false;

  // Apply adaptive threshold
  // create a click window:
    
  response.pointsResp.points.clear();
  int imageCount(0);
  // fill the blobs.
  // @Todo, add memory for the point selection.
  bool oldPtList(true);
  while (true)
  {
    cv::Mat displayImage(localImage.clone());

    for (int ind(0); ind < ptList.points.size(); ind++)
    {
      cv::circle(displayImage, cv::Point(ptList.points[ind].x, ptList.points[ind].y), 7, cv::Scalar(255, 0, 0), 2);
    }

    imshow("Selectable Points", displayImage);
    char keyIn = cv::waitKey(50);
    if (imagePt.x > 0)
    {
      if (oldPtList)
      {
        oldPtList = false;
        ptList.points.clear();
      }
      geometry_msgs::Point32 localPt;
      localPt.x = static_cast<float> (imagePt.x);
      localPt.y = static_cast<float> (imagePt.y);
      localPt.z = 0.0;
      response.pointsResp.points.push_back(localPt);
      ptList.points.push_back(localPt);
      imageCount++;

      imagePt.x = -1;
    }
    // if the Esc key is pressed, break out.
    if (keyIn == 27 || imageCount >= request.requestedPoints) break;

    // reuse the previous point list.
    if (keyIn == 'r' && oldPtList)
    {
      ROS_INFO("Re-using previous point set \n");
      for (int ind(0); ind < ptList.points.size(); ind++)
      {
        response.pointsResp.points.push_back(ptList.points[ind]);
      }
      break;
    }
  }

  ROS_INFO("Finished the acquiring the point list. \n");
  cv::Mat blank(cv::Mat::zeros(50, 50, CV_8UC1));
  imshow("Selectable Points", blank);
  cv::waitKey(10);
  // Merge blob to label image
  if (response.pointsResp.points.size() > 0)
  {
    return true;
  }
  else return false;
}


//updates the local image.
void ManualLabeling::newImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
   localImage =  cv_bridge::toCvShare(msg, "bgr8")->image.clone();
   imageRdy = true;
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg ->encoding.c_str());
  }

  ROS_INFO("Finished processing the input image");
}

