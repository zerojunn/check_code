#include "DroneSearch.h"

DroneSearch::DroneSearch(mavsdk::Mavsdk &mavsdk, std::string port) {
    connection_result = mavsdk.add_any_connection(port);
    if(connection_result != mavsdk::ConnectionResult::Success){
        std::cerr << "Connection failed : " << connection_result << "\n";
        return;
    }
}

std::shared_ptr<mavsdk::System> DroneSearch::get_system(mavsdk::Mavsdk &mavsdk) {
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
    auto fut = prom.get_future();

    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    return fut.get();
}

void DroneSearch::getPlugin(mavsdk::Mavsdk &mavsdk) {
    auto system = get_system(mavsdk);
    calibration_ = new mavsdk::Calibration{system};
    telemetry_ = new mavsdk::Telemetry{system};
    action_ = new mavsdk::Action{system};
    info_ = new mavsdk::Info{system};
    offboard_ = new mavsdk::Offboard{system};
}