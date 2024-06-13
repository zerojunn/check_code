#include "ArucoImageController.h"

ArucoImageController::ArucoImageController(int argc, char **argv, DroneControl *vehicle) : UserDefined(vehicle)
    , time_last(0), time_to_wait(0.1) {
    if(argc > 1 && argv[1] != nullptr){ id = std::stoi(argv[1]); }
    else { id = -1; }

    frameFromCamera();
    ros::init(argc, argv, "highroad_node");

    while(vehicle->inAir()){
    //while(1){
        cv::Mat frame;
        cap.read(frame);
        bridge_RosCv(&frame);
        }
}

void ArucoImageController::frameFromCamera() {
    cap = cv::VideoCapture(0);
    if(!cap.isOpened())
        std::cerr << "camera open failed" << std::endl;
        
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

}


void ArucoImageController::bridge_RosCv(cv::Mat *frame) {
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::Image>("/camera/color/camera_new", 1);
    typeCasting(frame);
}

void ArucoImageController::typeCasting(cv::Mat *frame) {
    sensor_msgs::ImageConstPtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", msg_receiver(frame)).toImageMsg();
    pub.publish(msg_);
}


cv::Mat ArucoImageController::msg_receiver(cv::Mat *frame) {
    if (frame->empty()) {
        std::cerr << "Camera Sensor Open failed!!" << std::endl;
        vehicle->land();
        return *frame;
    }

    centerPoint = cv::Point2f(frame->cols/2, frame->rows/2);
    dic[0] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
    dic[1] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    dic[2] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    dic[3] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
    dic[4] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    time_ = time(NULL);


    if(time_ - time_last > time_to_wait) {
    /**
      cv::aruco::detectMarkers(*frame, dic[4], corners, ids);
        if(ids.size()>0){
          LastPoint = getCenterPoint(corners, 0);
          cv::aruco::drawDetectedMarkers((*frame), corners);
          switchStep(frame);
        }
        else{ 
    **/
        for(int d = 0; d < 5; d++){
        cv::aruco::detectMarkers(*frame, dic[d], corners, ids);
        try {
            if (ids.size() > 0) {
                targetPoint[d] = getCenterPoint(corners, 0);
                if(d==4 and ids.size()>0){
                    LastPoint = getCenterPoint(corners, 0);
                }else{
                    LastPoint = getLastPoint(d,targetPoint[d]);
                }
                for (int i = 0; i < ids.size(); i++) {
                        cv::aruco::drawDetectedMarkers((*frame), corners);
                        switchStep(frame);
                }
            }else{
                cv::aruco::detectMarkers(*frame, dic[4], corners, ids);
                if(ids.size()>0){
                    LastPoint = getCenterPoint(corners, 0);
                    cv::aruco::drawDetectedMarkers((*frame), corners);
                    switchStep(frame);
                }
            }
        } catch (int exception) {
            std::cout << "target likely not found" << std::endl;
        }//catch
        }//for
        //}//else
    }

    return *frame;
}






