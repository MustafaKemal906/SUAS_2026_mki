cmake_minimum_required(VERSION 3.16)

project(pr-2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)




# GEREKLİ PAKETLER
find_package(MAVSDK REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
        include
        src/classes
        ${OpenCV_INCLUDE_DIRS}
)

# ADD EXECURABLE

add_executable(mission
        src/mission.cpp
        src/classes/CoveragePlanner.cpp
        src/classes/DroneController.cpp
)
target_link_libraries(mission
        PRIVATE
        MAVSDK::mavsdk
        ${OpenCV_LIBS}
)