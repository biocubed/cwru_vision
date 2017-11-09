/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Case Western Reserve University
 *    Russell C Jackson <rcj33@case.edu>
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
 */

#include <vector>
#include <ros/ros.h>
#include "cwru_opencv_common/circle_detection.h"

using cv::RotatedRect;
using cv::Mat;
using cv::Point;
using cv::Point2f;
using cv::Point2d;
using cv::Scalar;
using cv::Size2f;
using cv::MORPH_ELLIPSE;
using cv::MORPH_OPEN;
using cv::Size;
using cv::FLOODFILL_FIXED_RANGE;
using cv::FLOODFILL_MASK_ONLY;
using cv::Rect;
using cv::cvtColor;
using cv::Range;

cv::RotatedRect fillEllipseBW(const cv::Mat & inputImg, cv::Point seedPt)
{
  std::vector< std::vector<Point> > contours;


  if (seedPt.x > inputImg.cols || seedPt.y > inputImg.rows || seedPt.x < 0 || seedPt.y < 0)
  {
    return RotatedRect(Point2f(-1, -1), Size2f(-1, -1), 0);
  }

  int newMaskVal = 255;
  Scalar newVal = Scalar(120, 120, 120);

  int connectivity = 8;
  int flags = connectivity | (newMaskVal << 8) | FLOODFILL_FIXED_RANGE | FLOODFILL_MASK_ONLY;

  int lo = 20;
  int up = 10;

  Mat mask2 = Mat::zeros(inputImg.rows + 2, inputImg.cols + 2, CV_8UC1);
  floodFill(inputImg, mask2, seedPt, newVal, 0, Scalar(lo, lo, lo), Scalar(up, up, up), flags);
  Mat mask = mask2(Range(1, mask2.rows - 1), Range(1, mask2.cols - 1));


  Mat elementOpen = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
  Mat elementClose = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));

  Mat openMask;
  Mat closeMask;

  morphologyEx(mask, openMask, MORPH_OPEN, elementOpen);
  // morphologyEx(openMask, closeMask, MORPH_CLOSE, elementClose);
  // outputImg = closeMask.clone();

/*#ifdef DEBUGCATHSEG
    //imshow( "Open Mask", openMask );
    //imshow( "Close Mask", closeMask );

    //waitKey(0);

    //destroyWindow("Open Mask");
    //destroyWindow("Close Mask");

#endif*/

  findContours(openMask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  if (contours[0].size() > 5)
  {
    RotatedRect outputRect = fitEllipse(Mat(contours[0]));
    return outputRect;
  }

  RotatedRect outputRect(Point2f(-1, -1), Size2f(-1, -1), 0);
  return outputRect;
}

namespace cv_circle
{

void ellipseError(const RotatedRect & originalEllipse, const RotatedRect & newEllipse, Point2d& offset , double& angle)
{
  // compute the 4 points.
  Point2f origList[4];
  Point2f newList[4];

  originalEllipse.points(origList);
  newEllipse.points(newList);
  /*
   * @todo(rcj) Finish this function
   */
}

// Optimizes a guess of the ellipse
void optimizeEllipseBW(const cv::Mat & inputImage, cv::RotatedRect &ellipseIn, int padding)
{
  Rect imageRect(Point(0, 0), inputImage.size());

  Rect boundRect(ellipseIn.boundingRect());

  boundRect += Size(padding, padding);
  boundRect -= Point(padding, padding);

  boundRect &= imageRect;

  Mat subImg(inputImage(boundRect));
  Mat grayImg_, grayImg;

  //  Make the image gray
  cvtColor(subImg, grayImg_, CV_BGR2GRAY);

  //  perform a quick blur.
  GaussianBlur(grayImg_, grayImg, Size(7, 7) , 1.5, 1.5);

  Mat diffX, diffY;

  Scharr(grayImg, diffX, CV_32F, 1, 0);
  Scharr(grayImg, diffY, CV_32F, 0, 1);

  // the grad mag.
  Mat gradMag(diffX.mul(diffX)+diffY.mul(diffY));

  // not done yet

  // figure out the ellipse rendering in order generate the gradient.

  // iterate through the ellipse points.
  Mat ellipseDx, ellipseDy;

  Scharr(gradMag, ellipseDx, CV_32F, 1, 0);
  Scharr(gradMag, ellipseDy, CV_32F, 0, 1);
  // for each point on the drawn ellipse, try to improve its X and Y component. (only change the center.x and y.)

  Mat ellipseImg(boundRect.size(), CV_32FC1);

  RotatedRect tempEllipse(ellipseIn);
  tempEllipse.center.x -= boundRect.tl().x;
  tempEllipse.center.y -= boundRect.tl().y;

  // begin while true loop.

  ellipse(ellipseImg, tempEllipse, Scalar(1, 1, 1), 4);

  Mat prodX = ellipseImg.mul(ellipseDx);
  Mat prodY = ellipseImg.mul(ellipseDy);

  // optimize the ellipse by performing a weighted summation.
  int totalpixels = boundRect.area();

  Point2f offset(0.0, 0.0);

  for (int index(0); index < totalpixels; index++ )
  {
    if (ellipseImg.at<float>(index) != 0)
    {
      // Compute the gradiant etc.
      int x(index %  boundRect.width);
      int y(index / (boundRect.height));

      float xp = static_cast<float> (x-tempEllipse.center.x);
      float yp = static_cast<float> (y-tempEllipse.center.y);

      float norm = sqrt(xp*xp + yp*yp);

      float prod = prodX.at<float>(index) * xp/norm + prodY.at<float>(index) * yp/norm;

      offset.x += prod * xp;
      offset.y += prod * yp;
    }
  }
  // when finished, look at the results.
#ifdef DEBUGCATHSEG
  double maxVal(0);
    minMaxLoc(gradMag, NULL, &maxVal);
    Mat disp = gradMag*(1/maxVal);
    imshow("ellipse",  gradMag);

    ROS_OUT("The offset is < %f , %f >", offset.x, offset.y);

    waitKey(0);

    destroyWindow("ellipse");
    // destroyWindow("Close Mask");
#endif
  // Mat ellipseRend;
}

// render Ellipse Grad
void drawEllipseGrad(Mat & imageX, Mat & imageY, const RotatedRect &ellipse, double gaussVar, int gaussSide)
{
  // The goal of this function is to not simply draw the ellipse but to also incorporate the tangent information.
  double theta(0.0);
  while (theta < 6.28)
  {
    // The point and derivatives are not correct.
    // Point2d curvePt(ellipse.center.x+ellipse.size.width*cos(theta+ellipse.angle*3.14/180.0),
    //    ellipse.center.y+ellipse.size.height*sin(theta+ellipse.angle*3.14/180.0));

    // Point2d curveDPt(ellipse.size.width*cos(theta+ellipse.angle*3.14/180.0),
    //    ellipse.center.y+ellipse.size.height*sin(theta+ellipse.angle*3.14/180.0));
    break;
  }
}

};  // namespace cv_circle
