set(communication_headers
	BluetoothPort.hpp
)

set(communication_sources
	BluetoothPort.cpp
)

add_library(Communication STATIC
    ${communication_headers}
    ${communication_sources} 
)

set_target_properties(Communication PROPERTIES LINKER_LANGUAGE CXX CMAKE_CXX_STANDARD 17)