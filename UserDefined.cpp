#include <ros/init.h>
#include "UserDefined.h"
#include "DroneControl.h"

UserDefined::UserDefined(DroneControl *vehicle) : vehicle(vehicle)
        , markerSize(80), Switches(1), testpoint(5), highp(-0.3), yawcheck(false)
        , camMatrix((cv::Mat_<double>(3, 3) <<
                                            530.8269276712998, 0.0, 320.5, 0.0, 530.8269276712998, 240.5, 0.0, 0.0, 1.0))
        , distCoeffs((cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0)){
        dic[0] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
        dic[1] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
        dic[2] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
        dic[3] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
        dic[4] = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);}

void timesleep(int time, int& Switches) {
    std::this_thread::sleep_for(std::chrono::seconds(time));
    Switches = 3;
}


void UserDefined::switchStep(cv::Mat *frame) {
    distancePoint = LastPoint-centerPoint;
    // float currentAltitude = vehicle->getAltitude();

    switch(Switches){
        case 1 :
          std::cerr << "case - 1" << std::endl;
            cv::line(*frame, cv::Point(centerPoint), cv::Point(LastPoint), cv::Scalar(0, 0, 255), 5);
            if(testpoint == 5){
                //qusrudcheck
                checkdata = getSamesign(distancePoint.x,distancePoint.y);
                movepointx1 = fabs(distancePoint.x*5+distancePoint.x+1);
                movepointy1 = fabs(distancePoint.y*5+distancePoint.y+1);

                if(checkdata)
                    setSpace((distancePoint.x/movepointx1),(distancePoint.y/movepointy1)*-1,0.0);
                else
                    setSpace((distancePoint.x/movepointx1)*-1,(distancePoint.y/movepointy1),0.0);
            }
            if(40>fabs(distancePoint.x) and 40>fabs(distancePoint.y)){
                std::cerr << "setting done" << std::endl;
                testpoint = 0;
                //5m���� �Ҷ� ���� 3����
                Switches = 2;
            }

            break;
        case 2 :
        std::cerr << "case - 2" << std::endl;
            if(yawcheck == false){
                setSpace(0,0,0);}
            //cv::aruco::detectMarkers(*frame, dic[0], corners, ids);
            // cv::aruco::drawDetectedMarkers((*frame), corners);
            if(ids.size()>0){
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                std::cerr << "get marker" << std::endl;
                //cv::circle(*frame, corners[0][0],1,cv::Scalar(0,0,255),3);
                //cv::circle(*frame, corners[0][1],1,cv::Scalar(0,255,0),5);
                //cv::circle(*frame, corners[0][2],1,cv::Scalar(255,0,0),3);
                //cv::circle(*frame, corners[0][3],1,cv::Scalar(0,0,0),5);               
                if(yawcheck == false and ids[0]==352){
                    std::cerr << "turning start" << std::endl;
                    yawcheck = true;
                    if(abs(corners[0][3].x-corners[0][0].x)<2 and abs(corners[0][2].y-corners[0][3].y)<2 and corners[0][2].y<corners[0][1].y and corners[0][2].x<corners[0][3].x){
                        std::cerr << "yaw setting done :"<< abs(corners[0][1].x)<< "and" << abs(corners[0][2].x) << std::endl;
                        //yawcheck = true;
                        Switches = 3;
                    }
                    else{
                     //newcenter = (corners[0][0].x+corner[0][1].x)
                    
                    
                     a = (corners[0][2].x+corners[0][3].x)/2;
                     b = (corners[0][2].y+corners[0][3].y)/2;
                     x = (corners[0][0].x+corners[0][1].x+corners[0][2].x+corners[0][3].x)/4;
                     y = (corners[0][0].y+corners[0][1].y+corners[0][2].y+corners[0][3].y)/4;
                     //std::cerr << "corners 0 :"
                        //<< corners[0][0]<< "corners 1 : " 
                        //<< corners[0][1]<< "corners 2 : " 
                        //<< corners[0][2]<< "corners 3 : " 
                        //<< corners[0][3]<<std::endl;

                        atana = std::atan2((a-x),(b-y));
                        atana2 = atana * (180/3.141592);
                        std::cerr <<"atana : " <<atana2 << std::endl;
                        std::cerr <<corners[0][3]<<atana2 << std::endl;
                        std::cerr <<corners[0][0]<<atana2 << std::endl;
                        //vehicle->move_vel_byBody(0,0,0,atana2/5);
                        
                        if(atana2 < 0){
                            //setYawPnP(-15);
                            vehicle->move_vel_byBody(0,0,0,-30);
                            //std::cerr << "turning" << atana2 << std::endl;
                        }
                        else{
                            //setYawPnP(15);
                            vehicle->move_vel_byBody(0,0,0,30);
                            std::cerr << "turning" << atana2 << std::endl;
                        }
                        
                    }//else
                }//if
                else{
                  std::cerr << "first : not found marker"<< std::endl;
                
                }
            if(abs(corners[0][3].x-corners[0][0].x)<2 and abs(corners[0][2].y-corners[0][3].y)<2 and corners[0][2].y<corners[0][1].y and corners[0][2].x<corners[0][3].x){
                std::cerr << "yaw setting done :"<< abs(corners[0][1].x)<< "and" << abs(corners[0][2].x) << std::endl;
                //yawcheck = true;
                Switches = 3;
            }
            }else{
                std::cerr << "second : not found marker"<< std::endl;
            }
            break;
        case 3 :
        std::cerr << "case - 3" << std::endl;
            //cv::aruco::detectMarkers(*frame, dic[4], corners, ids);
           // std::cerr << ids[0] << std::endl;
            //if (ids.size()>0 and yawcheck==false){
              //  std::this_thread::sleep_for(std::chrono::seconds(2));
                //std::cerr << "find yaw gogo" << std::endl;
                //Switches = 2;
            //}
            
            if (ids.size() > 0){
              if(ids[0]!=72){
                LastPoint = centerPoint;
              }
            //    LastPoint = getCenterPoint(corners, 0);
                
            //    distancePoint = LastPoint-centerPoint;
            //    }
            
            // std::cerr << distancePoint << std::endl;
            movepointx1 = fabs(distancePoint.x*10+distancePoint.x+1);
            movepointy1 = fabs(distancePoint.y*10+distancePoint.y+1);

            //cv::line(*frame, cv::Point(centerPoint), cv::Point(LastPoint), cv::Scalar(0, 0, 255), 5);
            
            checkdata = getSamesign(distancePoint.x,distancePoint.y);
            can_move = abs(distancePoint.x)+abs(distancePoint.y);
            // std::cerr << can_move << std::endl;

            if(can_move>300){
              movepointx1 = fabs(distancePoint.x*10+distancePoint.x+1);
              movepointy1 = fabs(distancePoint.y*10+distancePoint.y+1);
              if(checkdata)
                setSpace((distancePoint.x/movepointx1),(distancePoint.y/movepointy1)*-1,highp);
              else
                setSpace((distancePoint.x/movepointx1)*-1,(distancePoint.y/movepointy1),highp);
            }else{
              if(can_move>30){
                if(checkdata)
                  setSpace((distancePoint.x/movepointx1),(distancePoint.y/movepointy1)*-1,highp);
                else
                  setSpace((distancePoint.x/movepointx1)*-1,(distancePoint.y/movepointy1),highp);
              }
              else{
                setSpace(0,0,highp);
                }
              }
            // print(getAltitude());
            // if(fabs(vehicle->getAltitude())<0.3){
            //     std::cerr << vehicle->getAltitude() << std::endl;
            //     Switches = 4;
            //     break;
                // }
            }
            break;
        case 4 :
            std::cerr << "This is case 4" << std::endl;
            vehicle->land();
            std::cerr << "finish" << std::endl;
            std::quick_exit(0);
        default :
            try{
                ros::shutdown();
            } catch (std::system_error){ }
    }
}

cv::Point2f UserDefined::getCenterPoint(std::vector<std::vector<cv::Point2f>> corners, int i) {
    float avg_x = ( static_cast<int>(corners[i][0].x) + static_cast<int>(corners[i][1].x) + static_cast<int>(corners[i][2].x)
                    + static_cast<int>(corners[i][3].x) ) / 4;
    float avg_y = ( static_cast<int>(corners[i][0].y) + static_cast<int>(corners[i][1].y) + static_cast<int>(corners[i][2].y)
                    + static_cast<int>(corners[i][3].y) ) / 4;;
    return cv::Point2f(avg_x, avg_y);
}

double UserDefined::getDistancePnP(cv::Point2f p1, cv::Point2f p2) {
    return std::sqrt(std::pow(p1.x-p2.x, 2) + std::pow(p1.y-p2.y, 2));
}

cv::Point2f UserDefined::getLastPoint(int i, cv::Point2f point){

    int check = 0, pointx = 0, pointy = 0;
    float avg_pointx, avg_pointy;

    switch(i){
        case 0:
            point1 = point;
            break;
        case 1:
            point2 = point;
            break;
        case 2:
            point3 = point;
            break;
        case 3:
            point4 = point;
            break;
        case 4:
            break;
    }

    if(point1.x){
        check += 1;
        pointx = pointx + point1.x;
        pointy = pointy + point1.y;
    }
    if(point2.x){
        check += 1;
        pointx = pointx + point2.x;
        pointy = pointy + point2.y;
    }
    if(point3.x){
        check += 1;
        pointx = pointx + point3.x;
        pointy = pointy + point3.y;
    }
    if(point4.x){
        check += 1;
        pointx = pointx + point4.x;
        pointy = pointy + point4.y;
    }
    
    avg_pointx = (pointx / check);
    avg_pointy = (pointy / check);


    return cv::Point2f(avg_pointx,avg_pointy);
}

double UserDefined::getGradientPnP(cv::Point2f p1, cv::Point2f p2) {
    return (p2.y-p1.y)/(p2.x-p1.x);
}

bool UserDefined::getSamesign(float x, float y) {
    if((x>=0 and y>=0) or (x<0 and y<0))
        return true;
    return false;
}

double UserDefined::getAnglePnP(cv::Point2f p1, cv::Point2f p2) {
    try{
        cv::Point2f tempPoint(p1.x+0.01, 1);
        double m1 = getGradientPnP(p1, p2);
        double m2 = getGradientPnP(p1, tempPoint);
        if(m2 > 0) m2 += (M_PI / 2);
        double angR = std::atan((m2-m1)/(1+m2*m1));
        double angD = std::trunc(angR * 180.0 / M_PI + 0.5);
        if(!std::isnan(angD)) return angD;
        else return -1.1111;
    } catch (std::exception& e){}
}

//thread
auto async_yaw = [](DroneControl* vehicle, double degree) { vehicle->move_vel_byBody(0,0,0,degree); };
auto run_async_yaw = [](DroneControl* vehicle, double degree) { std::thread async_yaw_thread(async_yaw, vehicle, degree); async_yaw_thread.detach(); };

auto async_move = [](DroneControl* vehicle, double x, double y, double s) { vehicle->move_vel_byBody(-x,-y,s,0); };
auto run_async_move = [](DroneControl* vehicle, double x, double y, double s)
{ std::thread async_move_thread(async_move, vehicle, -x, -y, s); async_move_thread.detach(); };

auto land_start = [](DroneControl* vehicle) { vehicle->land(); };

void UserDefined::land_end() {
    land_start(vehicle);
}

void UserDefined::drone_end() {
    drone_end();
}
void UserDefined::setSpace(double x, double y, double s) {
    run_async_move(vehicle, x, y, s);
}
void UserDefined::setYawPnP(double degree) {
    run_async_yaw(vehicle, degree);
}
