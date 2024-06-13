#include "DroneControl.h"
DroneControl::DroneControl(mavsdk::Mavsdk &mavsdk, std::string port) : DroneSearch(mavsdk, port) {
    getPlugin(mavsdk);
}

bool DroneControl::inAir() {
    if(telemetry_->in_air())
        return true;
    else
        return false;
}

float DroneControl::getAltitude() {
    return telemetry_->position_velocity_ned().position.down_m;
}

bool DroneControl::armable() {
    if(!telemetry_->in_air()){
        while(telemetry_->health_all_ok() != true){
            std::cout << "Vehicle is now armable" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    return telemetry_->health_all_ok();
}

bool DroneControl::arm() {
    const mavsdk::Action::Result arm_result = action_->arm();
    if(arm_result != mavsdk::Action::Result::Success){
        std::cerr << "Arming failed : " << arm_result << std::endl;
        std::quick_exit(0);
    }
    return true;
}

bool DroneControl::takeoff(const float takeoff_altitude) {
    action_->set_takeoff_altitude(takeoff_altitude);
    action_->takeoff();
    //while(true) {
        //float current_altitude = telemetry_->position_velocity_ned().position.down_m * -1;
        //std::cout << current_altitude << std::endl;
        //if (current_altitude >= 0.8 * takeoff_altitude) {
            //break;
        //}
        //std::this_thread::sleep_for(std::chrono::seconds(1));
    //}
    
    std::this_thread::sleep_for(std::chrono::seconds(7));
    std::cout << "Target altitude reached!" << std::endl;
    return true;
}

bool DroneControl::land() {
    action_->land();
    // while(telemetry_->in_air()){
    //     float current_altitude = telemetry_->position_velocity_ned().position.down_m * -1;
    //     // std::cout << current_altitude << std::endl;
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    // }
    std::cout << "Target altitude reached!" << std::endl;
    return true;
}


bool DroneControl::RTL(const float RTL_altitude) {
    action_->set_return_to_launch_altitude(RTL_altitude);
    action_->return_to_launch();
    while(telemetry_->in_air()){
        float current_altitude = telemetry_->position_velocity_ned().position.down_m * -1;
        std::cout << current_altitude << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Target altitude reached!" << std::endl;
    return true;
}

bool DroneControl::danger() {
    action_->kill(); return true;
}

bool DroneControl::move_vel_byBody(float north, float east, float down, float yaw) {
    if(down >0)
        down = - down;
    else
        down *= -1;
    offboard_->set_velocity_body({north, east, down, yaw});
    offboard_->start();
    return true;
}