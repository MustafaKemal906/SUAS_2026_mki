#ifndef COVERAGEPLANNER_H
#define COVERAGEPLANNER_H

#include <vector>
#include <string>

// Projenin tek Waypoint tanımı burasıdır.
struct Waypoint {
    double latitude;
    double longitude;
    float altitude;
    float speed;
};

class CoveragePlanner {
public:
    CoveragePlanner();

    // JSON dosyasından rotayı okuyup Waypoint vektörü döndürür
    std::vector<Waypoint> load_path_from_json(const std::string& filename);
};

#endif // COVERAGEPLANNER_H