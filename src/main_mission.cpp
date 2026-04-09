// std libs
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <cmath>

// mavsdk libs
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

// --- GÖREV NOKTASI OLUŞTURUCU ---
Mission::MissionItem make_mission_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through = true)
{
    Mission::MissionItem new_item{};
    new_item.latitude_deg = latitude_deg;
    new_item.longitude_deg = longitude_deg;
    new_item.relative_altitude_m = relative_altitude_m;
    new_item.speed_m_s = speed_m_s;
    new_item.is_fly_through = is_fly_through;

    // Haritalama için kamera aşağı (-90) bakar
    new_item.gimbal_pitch_deg = -90.0f;
    new_item.gimbal_yaw_deg = 0.0f;
    new_item.camera_action = Mission::MissionItem::CameraAction::None;

    return new_item;
}

// Metreyi enlem (latitude) derecesine çevirir
double meters_to_deg_lat(double meters) {
    return meters / 111111.0;
}

// Metreyi boylam (longitude) derecesine çevirir (Kosinüs düzeltmeli)
double meters_to_deg_lon(double meters, double current_lat) {
    return meters / (111111.0 * std::cos(current_lat * M_PI / 180.0));
}

int main()
{
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};

    // 1. BAĞLANTI
    const std::string port_url = "udp://:14540";
    std::cout << "Baglanti bekleniyor: " << port_url << std::endl;

    ConnectionResult connection_result = mavsdk.add_any_connection(port_url);
    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Baglanti Hatasi: " << connection_result << std::endl;
        return 1;
    }

    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Otopilot bulunamadi!\n";
        return 1;
    }
    std::cout << "Sistem Bulundu!\n";

    auto action = Action{system.value()};
    auto mission = Mission{system.value()};
    auto telemetry = Telemetry{system.value()};

    // 2. GPS VE KONUM BEKLEME
    std::cout << "GPS verisi bekleniyor...\n";
    while (!telemetry.health_all_ok()) {
        sleep_for(seconds(1));
    }
    std::cout << "Sistem hazir!\n";

    // 3. MEVCUT KONUMU AL
    Telemetry::Position start_pos = telemetry.position();
    double home_lat = start_pos.latitude_deg;
    double home_lon = start_pos.longitude_deg;

    std::cout << "Mevcut Konum (Kalkis Noktasi) Alindi: " << home_lat << ", " << home_lon << std::endl;

    mission.clear_mission();

    std::vector<Mission::MissionItem> mission_items;

    float altitude = 70.0f; // 70 metre yükseklik
    float speed = 5.0f;     // 5 m/s hız

    // Haritalama Alanı Parametreleri
    double sweep_length_m = 150.0;
    double line_spacing_m = 30.0;
    int num_lines = 5;

    double sweep_len_deg = meters_to_deg_lat(sweep_length_m);
    double spacing_deg_lon = meters_to_deg_lon(line_spacing_m, home_lat);

    std::cout << "Haritalama rotasi hesaplaniyor...\n";

    // İlk Nokta (Kalkıştan sonra hizalanılacak ilk koordinat)
    mission_items.push_back(make_mission_item(home_lat, home_lon, altitude, speed));

    // Gel-Git (Lawnmower) Şeritleri
    for (int i = 0; i < num_lines; ++i) {
        double current_lon = home_lon + (i * spacing_deg_lon);
        double next_lon = home_lon + ((i + 1) * spacing_deg_lon);

        if (i % 2 == 0) {
            mission_items.push_back(make_mission_item(home_lat + sweep_len_deg, current_lon, altitude, speed));
            if (i < num_lines - 1) {
                mission_items.push_back(make_mission_item(home_lat + sweep_len_deg, next_lon, altitude, speed));
            }
        } else {
            mission_items.push_back(make_mission_item(home_lat, current_lon, altitude, speed));
            if (i < num_lines - 1) {
                mission_items.push_back(make_mission_item(home_lat, next_lon, altitude, speed));
            }
        }
    }

    // Rotanın sonuna, kalkış noktasına havadan (70m) dönüş komutu
    std::cout << "Rotanin sonuna baslangic noktasina donus eklendi.\n";
    mission_items.push_back(make_mission_item(home_lat, home_lon, altitude, speed));

    Mission::MissionPlan mission_plan{};
    mission_plan.mission_items = mission_items;

    if (mission.upload_mission(mission_plan) != Mission::Result::Success) {
        std::cerr << "❌ Gorev yuklenemedi!\n";
        return 1;
    }
    std::cout << "Gorev basariyla yuklendi!\n";

    // 4. ARM, TAKEOFF VE GÖREVİ BAŞLAT
    std::cout << "Arm ediliyor...\n";
    Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "❌ Arm Hatasi: " << arm_result << "\n(GPS tam oturmamis veya baska bir pre-flight hatasi olabilir)\n";
        return 1;
    }

    std::cout << "Kalkis (Takeoff) yapiliyor...\n";
    Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "❌ Takeoff Hatasi: " << takeoff_result << '\n';
        return 1;
    }

    // Dronun havalanması ve stabilite kazanması için bekleme süresi
    std::cout << "Dronun havalanmasi bekleniyor (7 saniye)...\n";
    sleep_for(seconds(7));

    std::cout << "Haritalama gorevi BASLATILIYOR...\n";
    Mission::Result start_result = mission.start_mission();
    if (start_result != Mission::Result::Success) {
        std::cerr << "❌ Gorev baslatma hatasi: " << start_result << '\n';
        return 1;
    }

    // 5. GÖREV TAKİBİ
    while (!mission.is_mission_finished().second) {
        sleep_for(seconds(1));
        auto progress = mission.mission_progress();
        std::cout << "Gorev: " << progress.current << " / " << progress.total << "\r" << std::flush;
    }

    std::cout << "\n✅ Haritalama gorevi bitti! Tam baslangic noktasindayiz, inis (Land) yapiliyor...\n";

    // Görev sonu doğrudan iniş (Zaten Home pozisyonunun tepesindeyiz)
    action.land();

    // İniş bitip motorlar kapanana (disarm) kadar bekle
    while (telemetry.armed()) {
        sleep_for(seconds(1));
    }
    std::cout << "Inis tamamlandi. Program kapaniyor.\n";

    return 0;
}
