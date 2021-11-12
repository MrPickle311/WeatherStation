project(WeatherStationTests 0.1)

set_target_properties(${PROJECT_NAME} 
	PROPERTIES
    CXX_STANDARD 17
    CMAKE_CXX_STANDARD_REQUIRED ON
    )

set(HEADERS_DIRS 
    ${PROJECT_SOURCE_DIR}/src/Processing
    ${PROJECT_SOURCE_DIR}/src/Devices/Sensor
	${PROJECT_SOURCE_DIR}/src/Devices/Engine
	${PROJECT_SOURCE_DIR}/src/Devices/Communication
    )

include_directories(${HEADERS_DIRS})

add_subdirectory(src)
add_subdirectory(tests)
