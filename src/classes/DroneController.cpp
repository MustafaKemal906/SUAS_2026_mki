#include "DroneController.h"
#include <iostream>
#include <chrono>
#include <thread>

// UDP ve Network kütüphaneleri
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

using std::chrono::seconds;
using std::this_thread::sleep_for;

// Yapıcı ve Yıkıcı Fonksiyonlar
DroneController::DroneController() :
    mavsdk_instance{mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}} {}

DroneController::~DroneController() {}

// --- BAGLANTI VE SISTEM FONKSIYONLARI ---

bool DroneController::connect(const std::string& port_url) {
    mavsdk::ConnectionResult connection_result = mavsdk_instance.add_any_connection(port_url);
    if (connection_result != mavsdk::ConnectionResult::Success) {
        return false;
    }

    std::cout << "Otopilot araniyor...\n";
    auto system_optional = mavsdk_instance.first_autopilot(3.0);

    if (!system_optional) {
        return false;
    }

    system = system_optional.value();
    std::cout << "✅ Otopilot Baglantisi Kuruldu!\n";

    action = std::make_unique<mavsdk::Action>(system);
    mission = std::make_unique<mavsdk::Mission>(system);
    telemetry = std::make_unique<mavsdk::Telemetry>(system);

    return true;
}

bool DroneController::wait_for_health() {
    if (!telemetry) return false;
    std::cout << "Sensor saglik durumu kontrol ediliyor...\n";

    while (true) {
        auto health = telemetry->health();
        if (health.is_global_position_ok && health.is_home_position_ok) {
            std::cout << "\n✅ GPS ve Home hazir!\n";
            break;
        }
        std::cout << "GPS: " << (health.is_global_position_ok ? "OK" : "YOK") << "\r" << std::flush;
        sleep_for(seconds(1));
    }
    return true;
}

// --- FOTOĞRAF TETIKLEYICI (UDP) ---

void DroneController::send_udp_trigger() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return;

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(5005);
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");

    const char* msg = "SNAP";
    sendto(sock, msg, strlen(msg), 0, (const struct sockaddr*)&servaddr, sizeof(servaddr));
    close(sock);
}

// --- GOREV YONETIMI ---

mavsdk::Mission::MissionItem DroneController::create_mission_item(const Waypoint& wp) {
    mavsdk::Mission::MissionItem new_item{};
    new_item.latitude_deg = wp.latitude;
    new_item.longitude_deg = wp.longitude;
    new_item.relative_altitude_m = wp.altitude;
    new_item.speed_m_s = wp.speed;
    new_item.is_fly_through = true;
    new_item.gimbal_pitch_deg = -90.0f; // Haritalama için aşağı bak
    return new_item;
}

bool DroneController::upload_mission(const std::vector<Waypoint>& waypoints) {
    mission->clear_mission();
    mavsdk::Mission::MissionPlan mission_plan{};
    for (const auto& wp : waypoints) {
        mission_plan.mission_items.push_back(create_mission_item(wp));
    }
    auto result = mission->upload_mission(mission_plan);
    return (result == mavsdk::Mission::Result::Success);
}

bool DroneController::arm_and_takeoff() {
    if (action->arm() != mavsdk::Action::Result::Success) return false;
    return (action->takeoff() == mavsdk::Action::Result::Success);
}

bool DroneController::start_mission() {
    return (mission->start_mission() == mavsdk::Mission::Result::Success);
}

void DroneController::wait_until_mission_finished() {
    int last_triggered_wp = -1;
    while (true) {
        auto mission_res = mission->is_mission_finished();
        if (mission_res.first == mavsdk::Mission::Result::Success && mission_res.second) break;

        auto progress = mission->mission_progress();

        // Yeni waypoint'e gelince tetikle
        if (progress.current >= 0 && progress.current != last_triggered_wp) {
            std::cout << "\n📸 Waypoint #" << progress.current << " tetikleniyor..." << std::endl;
            send_udp_trigger();
            last_triggered_wp = progress.current;
        }
        std::cout << "Ilerleme: " << progress.current << " / " << progress.total << "\r" << std::flush;
        sleep_for(std::chrono::milliseconds(200));
    }
}

void DroneController::land() {
    action->land();
}

std::pair<double, double> DroneController::get_current_position() {
    auto pos = telemetry->position();
    return {pos.latitude_deg, pos.longitude_deg};
}