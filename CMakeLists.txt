cmake_minimum_required(VERSION 3.10)

project(WeatherStationTests VERSION 0.1 LANGUAGES CXX)


set (CMAKE_CXX_STANDARD 17)

set(HEADERS_DIRS 
    ${PROJECT_SOURCE_DIR}/src/Processing
    ${PROJECT_SOURCE_DIR}/src/Devices/Sensor
	${PROJECT_SOURCE_DIR}/src/Devices/Engine
	${PROJECT_SOURCE_DIR}/src/Devices/Communication
    )

include_directories(${HEADERS_DIRS})

add_subdirectory(src)
add_subdirectory(tests)
