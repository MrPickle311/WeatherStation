set(sensor_headers
	HTS22.hpp
	LPS22.hpp
	I2CSensor.hpp
)

set(sensor_sources
	HTS22.cpp
	LPS22.cpp
	I2CSensor.cpp
)

add_library(Sensors STATIC
    ${sensor_headers}
    ${sensor_sources} 
)

set_target_properties(Sensors PROPERTIES LINKER_LANGUAGE CXX CMAKE_CXX_STANDARD 17)