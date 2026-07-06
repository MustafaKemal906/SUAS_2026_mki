#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

// Oluşturduğumuz classlar
#include "classes/DroneController.h"
#include "classes/CoveragePlanner.h"

using namespace std::chrono_literals;

int main() {
    std::cout << "--- SUAS Otonom Haritalama ve Fotoğraf Tetikleme Sistemi ---" << std::endl;

    DroneController controller;
    CoveragePlanner planner;

    // Load waypoints
    std::vector<Waypoint> mission_path = planner.load_path_from_json("mission_waypoints.json");

    if (mission_path.empty()) {
        std::cerr << "❌ HATA: Rota dosyası okunamadı veya boş!" << std::endl;
        return 1;
    }
    std::cout << "✅ Rota okundu. Toplam Waypoint: " << mission_path.size() << std::endl;

    // Connection
    if (!controller.connect("udp://:14540")) {
        std::cerr << "❌ HATA: Drone bağlantısı kurulamadı!" << std::endl;
        return 1;
    }

    // Health Check
    if (!controller.wait_for_health()) {
        std::cerr << "❌ HATA: Sensörler uçuş için hazır değil!" << std::endl;
        return 1;
    }

    // 4. Görevi Yükle (Upload Mission)
    if (!controller.upload_mission(mission_path)) {
        std::cerr << "❌ HATA: Görev otopilota yüklenemedi!" << std::endl;
        return 1;
    }

    std::cout << "✅ Görev başarıyla yüklendi!" << std::endl;

    // 5. Motorları Çalıştır ve Kalkış (Arm & Takeoff)
    std::cout << "🚀 Arm ediliyor ve kalkış yapılıyor..." << std::endl;
    if (controller.arm_and_takeoff()) {
        // Kalkıştan sonra drone'un stabilize olması için kısa bir bekleme
        std::cout << "Havalimanı güvenliği sağlanıyor (5 sn bekleme)..." << std::endl;
        std::this_thread::sleep_for(5s); // Süreyi 15 saniyeye çıkarın

        // 6. Haritalama Görevini Başlat
        std::cout << "📸 Haritalama görevi BASLATILIYOR..." << std::endl;
        if (controller.start_mission()) {
            /* KRİTİK NOKTA:
               Bu fonksiyon içinde her waypoint geçişinde
               send_udp_trigger() çağrılacak ve Python tarafına
               'SNAP' mesajı gidecektir.
            */
            controller.wait_until_mission_finished();
        } else {
            std::cerr << "❌ HATA: Görev başlatılamadı!" << std::endl;
        }
    } else {
        std::cerr << "❌ HATA: Kalkış başarısız!" << std::endl;
    }

    // 7. Görev Bitişi ve İniş (Land)
    std::cout << "✅ Görev bitti. İniş yapılıyor..." << std::endl;
    controller.land();

    std::cout << "🏁 Program başarıyla sonlandırıldı." << std::endl;
    return 0;
}
