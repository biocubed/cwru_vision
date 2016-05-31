/* 
 * projective_geometry.h
 * Copyright 2014 Russell Jackson
 * All rights reserved.
 */

/*
 * The functions declared in this file are meant to handle the projective geometry of the cameras
*/


#ifndef PROJECTIVE_GEOMETRY_H
#define PROJECTIVE_GEOMETRY_H


#include <ros/ros.h>

#include <iostream>
#include <opencv2/opencv.hpp>

#include "cwru_opencv_common/opencv_local.h"
#include <sensor_msgs/CameraInfo.h>
#include <string>


namespace cv_projective
{


class cameraProjectionMatrices
{
public:


    cameraProjectionMatrices(ros::NodeHandle&, const std::string& , const std::string& );

    cv::Mat getLeftProjectionMatrix() const;
    cv::Mat getRightProjectionMatrix() const;

private:

    void projectionSubscriptionCb(const sensor_msgs::CameraInfoConstPtr &, int );
    // left and right projective matrices
    bool stereo;
    cv::Mat P_l;
    cv::Mat P_r;

    // left and right subscribers
    ros::Subscriber subL;
    ros::Subscriber subR;


};



/** 
 * @brief Projects a point in 3d coords to a point in a camera image.
 *
 */
cv::Point2d reprojectPoint(const cv::Point3d &point, const cv::Mat &P,const cv::Mat& = cv::Mat(), const cv::Mat & = cv::Mat(), cv::OutputArray = cv::noArray());
void reprojectPoints(cv::InputArray spacialPoints, cv::OutputArray imagePoints, const cv::Mat &P, const cv::Mat & = cv::Mat(), const cv::Mat & = cv::Mat());




/** 
 * @brief Projects a point in 3d coords to a pair of points in a stereo camera system
 *
 */
cv_local::stereoCorrespondence reprojectPointStereo(const cv::Point3d &point, const cv::Mat &P_l, const cv::Mat &P_r, const cv::Mat & = cv::Mat(), const cv::Mat & = cv::Mat());

/** 
 * @brief Projects an array of points in 3d coords to a pair of points in a stereo camera system
 *
 */
void reprojectPointsStereo(cv::InputArray points, std::vector < cv_local::stereoCorrespondence > & ,const cv::Mat &P_l, const cv::Mat &P_r,const cv::Mat & = cv::Mat(), const cv::Mat & = cv::Mat() );
void reprojectPointsStereo(cv::InputArray points, cv::OutputArray points_left, cv::OutputArray points_right  ,const cv::Mat &P_l, const cv::Mat &P_r, const cv::Mat & = cv::Mat(),  const cv::Mat & = cv::Mat() );

/**
 * @brief Projects a point tangent (in 3d) as a tangent in the stereo camera images.
 */
cv_local::stereoCorrespondence reprojectPointTangent(const cv::Point3d &point,const cv::Point3d &pointDeriv,const cv::Mat & P_l , const cv::Mat & P_r );


/**
 * @brief Projects a tangent into the camera image space from 3d space.
 *
 * @param const cv::Point3d & the point in 3d space.
 * @param const cv::Point3d & the tangent in 3d space.
 * @param const cv::Mat & The camera projection matrix.
 *
 * @return cv::Point2d the equivilent tangent vector in the image space.
 */
cv::Point2d reprojectPointTangent(const cv::Point3d &point, const cv::Point3d &pointDeriv, const cv::Mat & P);


/**
 * @brief deprojects a point tangent from camera image space into stereo space.
 */
cv::Point3d deprojectStereoTangent(const cv_local::stereoCorrespondence &imagePoint,const cv_local::stereoCorrespondence &imagePointTangent,const cv::Mat & , const cv::Mat & );


/**
 * @brief deprojects a stereo point from a image pair into a 3d point
 */
cv::Point3d deprojectStereoPoint(const cv_local::stereoCorrespondence ,const  cv::Mat &P_l ,const cv::Mat &P_r);


/**
 * @brief deproject an array of points.
 */
void deprojectStereoPoints(const std::vector < cv_local::stereoCorrespondence > &inputArray , cv::OutputArray outputPointArray, const cv::Mat& P_l , const cv::Mat P_r);

/**
 * @brief compute the transform G, based on rvect, and tvect, along with an optional output jacobian.
 * 
 */
void transformJacobian(const cv::Mat &rvect ,const cv::Mat & tvect, cv::Mat & trans , cv::OutputArray trans_jac=cv::noArray());


/**
 * @brief transforms points from 1 frame to another using rvec and tvec.
 */
cv::Mat transformPoints(const cv::Mat &points,const cv::Mat& rvec, const cv::Mat &tvec, cv::OutputArray jac =cv::noArray());

/**
 * @brief deprojects the endpoints of 2 ellipses
 */
void deprojectEllipseEnds(const cv_local::rotatedRectStereoCorr &, cv::Point3d & , cv::Point3d & , const cv::Mat & , const cv::Mat &);

};


#endif

