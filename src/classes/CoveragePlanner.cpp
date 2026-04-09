#include "CoveragePlanner.h"
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp> // Az önce indirdiğimiz JSON kütüphanesi

// nlohmann::json için bir takma ad
using json = nlohmann::json;

CoveragePlanner::CoveragePlanner() {}

std::vector<Waypoint> CoveragePlanner::load_path_from_json(const std::string& filename) {
    std::vector<Waypoint> path;
    std::ifstream file(filename);

    std::cout << "\n[STEP 1 - C++ JSON LOAD] Dosya aciliyor: " << filename << std::endl;

    if (!file.is_open()) {
        std::cerr << "!!! HATA: Dosya bulunamadi! (Working Directory kontrol et)" << std::endl;
        return path;
    }

    json json_waypoints;
    file >> json_waypoints;

    int count = 0;
    for (const auto& wp : json_waypoints) {
        Waypoint temp_wp = {
            wp.at("latitude").get<double>(),
            wp.at("longitude").get<double>(),
            wp.at("altitude").get<float>(),
            wp.at("speed").get<float>()
        };
        path.push_back(temp_wp);

        // Her noktayı tek tek logla
        std::cout << "  -> Nokta #" << ++count << " Okundu: Lat=" << temp_wp.latitude
                  << ", Lon=" << temp_wp.longitude << std::endl;
    }

    std::cout << "[STEP 1] Toplam " << path.size() << " nokta basariyla hafizaya alindi.\n" << std::endl;
    return path;
}