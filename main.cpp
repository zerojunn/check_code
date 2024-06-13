#include <iostream>
#include "DroneControl.h"
#include "ArucoImageController.h"
//jenkins test
//jenkins test
//jenkins test
//jenkins test

int main(int argc, char* argv[])
{

    std::this_thread::sleep_for(std::chrono::seconds(2));
    mavsdk::Mavsdk mavsdk;
    DroneControl *vehicle = new DroneControl(mavsdk, "udp://:14550");
    if(vehicle->armable()){
        if(vehicle->arm()){
           std::this_thread::sleep_for(std::chrono::seconds(4));
           vehicle->takeoff(5.0);
        }
    }

    //vehicle->move_vel_byBody(2,0,0,0);
    ArucoImageController(argc, argv, vehicle);
    
    return 0;
}
