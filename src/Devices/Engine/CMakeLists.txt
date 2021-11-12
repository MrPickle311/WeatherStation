set(engine_headers
	Engines.hpp
)

set(engine_sources

)

add_library(Engine STATIC
    ${engine_headers}
    ${engine_sources} 
)

set_target_properties(Engine PROPERTIES LINKER_LANGUAGE CXX CMAKE_CXX_STANDARD 17)