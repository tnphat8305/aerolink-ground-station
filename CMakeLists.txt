cmake_minimum_required(VERSION 3.16)

project(DroneAutomationCpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets SerialPort Network)

add_executable(DroneAutomationCpp
    main.cpp
)

target_link_libraries(DroneAutomationCpp PRIVATE
    Qt6::Widgets
    Qt6::SerialPort
    Qt6::Network
)
