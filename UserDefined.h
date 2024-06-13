#ifndef HIGHROAD_APRECISIONLANDING_USERDEFINED_H
#define HIGHROAD_APRECISIONLANDING_USERDEFINED_H

#include <iostream>
#include <cmath>
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"


#include "DroneControl.h"

class UserDefined {
public:
    UserDefined(DroneControl *vehicle);
    ~UserDefined(){};

    void switchStep(cv::Mat *frame);

    cv::Point2f getCenterPoint(std::vector<std::vector<cv::Point2f>> corners, int i);
    cv::Point2f getLastPoint(int i, cv::Point2f point);

    double getDistancePnP(cv::Point2f p1, cv::Point2f p2);
    double getGradientPnP(cv::Point2f p1, cv::Point2f p2);
    double getAnglePnP(cv::Point2f p1, cv::Point2f p2);
    bool getSamesign(float x, float y);

    void setSpace(double x, double y, double s);
    void land_end();
    void drone_end();
    double getdetectPnP(int id);
    void setYawPnP(double degree);


protected:
    //UserDefined                                                   sd
    DroneControl                                                    *vehicle;

    cv::Ptr<cv::aruco::Dictionary>                                  dic[5];

    std::vector<std::vector<cv::Point2f>>                           corners;

    cv::Point2f                                                     centerPoint,
                                                                    targetPoint[5],
                                                                    point1, point2, point3, point4,
                                                                    LastPoint,
                                                                    distancePoint;

    cv::Point3f                                                     transVec;

    std::vector<int>                                                ids;

    int                                                             id,
                                                                    Switches,
                                                                    checkdg,
                                                                    testpoint,a,b,x,y,
                                                                    can_move;

    bool                                                            checkdata, yawcheck;

    std::vector<cv::Vec3d>                                          rvecs,
                                                                    tvecs;

    cv::Mat                                                         camMatrix,
                                                                    distCoeffs;


    float                                                           markerSize,atana,atana2,atana3,highp,movepointx1,movepointy1;

    double                                                          distancePnP,
                                                                    dgreePnP;

};


#endif //HIGHROAD_APRECISIONLANDING_USERDEFINED_H
