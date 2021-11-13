set(sensor_headers
	HTS22.hpp
	LPS22.hpp
	I2CSensor.hpp
)

set(sensor_sources
)

add_library(Sensors STATIC
    ${sensor_headers}
    ${sensor_sources} 
)

set_target_properties(Sensors PROPERTIES LINKER_LANGUAGE CXX CMAKE_CXX_STANDARD 17)