#ifndef DRONECONTROLLER_H
#define DRONECONTROLLER_H

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <memory>
#include <vector>
#include <string>

// Waypoint tanımını CoveragePlanner.h içerisinden güvenli bir şekilde alıyoruz
#include "CoveragePlanner.h"

class DroneController {
public:
    DroneController();
    ~DroneController();

    bool connect(const std::string& port_url);
    bool wait_for_health();
    bool upload_mission(const std::vector<Waypoint>& waypoints);
    bool arm_and_takeoff();
    bool start_mission();
    void wait_until_mission_finished();
    void land();

    std::pair<double, double> get_current_position();

private:
    // Python tetikleyici sinyalini gönderen gizli fonksiyon
    void send_udp_trigger();

    mavsdk::Mavsdk mavsdk_instance;
    std::shared_ptr<mavsdk::System> system;

    std::unique_ptr<mavsdk::Action> action;
    std::unique_ptr<mavsdk::Mission> mission;
    std::unique_ptr<mavsdk::Telemetry> telemetry;

    // Yardımcı fonksiyonlar
    mavsdk::Mission::MissionItem create_mission_item(const Waypoint& wp);
};

#endif // DRONECONTROLLER_H