#ifndef PRECISIONLANDING_DRONESEARCH_H
#define PRECISIONLANDING_DRONESEARCH_H

#include <iostream>
#include <future>
#include <chrono>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/info/info.h>
#include <mavsdk/plugins/calibration/calibration.h>

class DroneSearch {
public:
    DroneSearch(mavsdk::Mavsdk &mavsdk, std::string port);


    static std::shared_ptr<mavsdk::System> get_system(mavsdk::Mavsdk& mavsdk);
    void getPlugin(mavsdk::Mavsdk &mavsdk);

protected:
    mavsdk::Telemetry                                                       *telemetry_;
    mavsdk::Calibration                                                     *calibration_;
    mavsdk::Action                                                          *action_;
    mavsdk::Mission                                                         *mission_;
    mavsdk::Info                                                            *info_;
    mavsdk::Offboard                                                        *offboard_;

private:
    mavsdk::Mavsdk                                                          vehicle;
    mavsdk::ConnectionResult                                                connection_result;
    std::shared_ptr<mavsdk::System>                                         System_;
};


#endif //PRECISIONLANDING_DRONESEARCH_H
