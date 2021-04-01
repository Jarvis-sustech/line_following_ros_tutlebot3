/** MIT License
Copyright (c) 2017 Sudarshan Raghunathan
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*
*@copyright Copyright 2017 Sudarshan Raghunathan
*@file linedetect.hpp
*@author Sudarshan Raghunathan
*@brief Header file for class linedetect
*/

#pragma once
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "line_follower_turtlebot/pos.h"

/***
 * pd控制器基本思想：当误差较大时，减小线速度，增大角速度；当误差较小时，增大线速度，减小角速度
***/
class pdControl{
public:
    pdControl(float _Kp_a=0, float _Kd_a=0, float _umax=0, float _umin=0){
        this->Kp_v = 0;
        this->Kd_v = 0;
        this->Kp_a = _Kp_a;
        this->Kd_a = _Kd_a;
        this->umax = _umax;
        this->umin = _umin;
        this->lineVelocity = 0;
        this->angleVelocity = 0;
        this->is_recorded = false;
    }

    float setLineVelocity();
    float setAngleVelocity();

    float err;  // 偏差值，代表测量量与控制量的误差
    float last_err;  // 上一偏差值
    float Kp_v, Kd_v, Kp_a, Kd_a; // 线速度和角速度-pd控制器的比例和微分系数
    float lineVelocity, angleVelocity;  // 输出的线速度和角速度
    float umax, umin;  // 设置速度的上限与下限
    bool is_recorded;  // 是否为第一次记录
};

/**
*@brief Line Detect class contains all the functions for image procesing and direction publishing
*/
class LineDetect {
 public:
    cv::Mat img;  /// Input image in opencv matrix format
    cv::Mat img_filt;  /// Filtered image in opencv matrix format
    int dir;  /// Direction message to be published
/**
*@brief Callback used to subscribe to the image topic from the Turtlebot and convert to opencv image format
*@param msg is the image message for ROS
*@return none
*/
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    //void arcodeCallback(const line_follower_turtlebot::AlvarMarker& msg)
/**
*@brief Function that applies Gaussian filter in the input image 
*@param input is the image from the turtlebot in opencv matrix format
*@return Mat of Gaussian filtered image in opencv matrix format
*/
    cv::Mat Gauss(cv::Mat input);
/**
*@brief Function to perform line detection using color thresholding,image masking and centroid detection to publish direction 
*@param input is the Filtered input image in opencv matrix format
*@return int direction which returns the direction the turtlebot should head in
*/
    int colorthresh(cv::Mat input, pdControl& pd);

 private:
    cv::Scalar LowerYellow;
    cv::Scalar UpperYellow;
    cv::Mat img_hsv;
    cv::Mat img_gray;
    cv::Mat img_mask;
};


