#ifndef PRECISIONLANDING_ROSCONTROL_H
#define PRECISIONLANDING_ROSCONTROL_H

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv4/opencv2/opencv.hpp>

#include "UserDefined.h"
#include "DroneControl.h"

class ArucoImageController : public UserDefined {
public:
    //생성자 & 소멸자
    //Ros 초기화
    ArucoImageController(int argc, char* argv[], DroneControl *vehicle);
    ~ArucoImageController(){};

    void frameFromCamera();
    void bridge_RosCv(cv::Mat *frame);
    void typeCasting(cv::Mat *frame);
    cv::Mat msg_receiver(cv::Mat *frame);

private:
    ros::Publisher                                                  pub;
    cv::VideoCapture                                                cap;

    time_t                                                          time_;
    double                                                          time_last,
                                                                    time_to_wait;

};


#endif //PRECISIONLANDING_ROSCONTROL_H
