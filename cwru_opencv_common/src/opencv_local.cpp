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


// This file defines common useful functions (based on opencv) that are used throughout the
// programs.

#include <algorithm>
#include <vector>
#include "cwru_opencv_common/opencv_local.h"

namespace cv_local
{

int byteError(int a, int b)
{
  int  hueError1 = (256-a)%256+b;
  int  hueError2 = (256-b)%256+a;
  int  hueError3 = abs(a-b);
  return cv::min(cv::min(hueError1, hueError2), hueError3);
}

bool contourCompare(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2)
{
  // Right now the comparison is number of points
  // A beeter comparison might be area:
  double size1 = contour1.size();
  double size2 = contour2.size();
  // Descending size:
  if (size1 >  size2) return true;
  else return false;
}

int countOutputChannels(int conversion)
{
  switch (conversion)
  {
    case CV_BGR2BGRA:
    case CV_RGB2BGRA:
    case CV_BGRA2RGBA:
    case CV_BGR5652BGRA:
    case CV_BGR5552BGRA:
    case CV_BGR5652RGBA:
    case CV_BGR5552RGBA:
    case CV_GRAY2BGRA:
      return 4;
      break;

    case CV_BGR2YCrCb:
    case CV_RGB2YCrCb:
    case CV_BGR2XYZ:
    case CV_RGB2XYZ:
    case CV_BGR2HSV:
    case CV_RGB2HSV:
    case CV_BGR2Lab:
    case CV_RGB2Lab:
    case CV_BGR2Luv:
    case CV_RGB2Luv:
    case CV_BGR2HLS:
    case CV_RGB2HLS:
      return 3;
      break;

    case CV_BayerBG2BGR:
    case CV_BayerGB2BGR:
    case CV_BayerRG2BGR:
    case CV_BayerGR2BGR:

    case CV_BGRA2BGR:
    case CV_RGBA2BGR:
    case CV_RGB2BGR:
    case CV_BGR5652BGR:
    case CV_BGR5552BGR:
    case CV_BGR5652RGB:
    case CV_BGR5552RGB:
    case CV_GRAY2BGR:

    case CV_YCrCb2BGR:
    case CV_YCrCb2RGB:
    case CV_XYZ2BGR:
    case CV_XYZ2RGB:
    case CV_HSV2BGR:
    case CV_HSV2RGB:
    case CV_Lab2BGR:
    case CV_Lab2RGB:
    case CV_Luv2BGR:
    case CV_Luv2RGB:
    case CV_HLS2BGR:
    case CV_HLS2RGB:
      return 3;
      break;

    case CV_BGR2BGR565:
    case CV_BGR2BGR555:
    case CV_RGB2BGR565:
    case CV_RGB2BGR555:
    case CV_BGRA2BGR565:
    case CV_BGRA2BGR555:
    case CV_RGBA2BGR565:
    case CV_RGBA2BGR555:
    case CV_GRAY2BGR565:
    case CV_GRAY2BGR555:
      return 2;
      break;

    case CV_BGR2GRAY:
    case CV_BGRA2GRAY:
    case CV_RGB2GRAY:
    case CV_RGBA2GRAY:
    case CV_BGR5652GRAY:
    case CV_BGR5552GRAY:
      return 1;
      break;
    default:
      CV_Error(CV_StsBadFlag, "Unknown/unsupported color conversion code");
      return -1;
  }
}
};  // namespace cv_local
