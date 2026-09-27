cmake_minimum_required(VERSION 3.20)

project(TeleoperationCommandInterpolatorConsumer LANGUAGES CXX)

find_package(TeleoperationCommandInterpolator 1.0 REQUIRED)

add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE teleoperation::command_interpolator)
target_compile_features(consumer PRIVATE cxx_std_17)
