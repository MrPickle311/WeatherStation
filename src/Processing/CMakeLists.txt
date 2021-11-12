set(processing_headers
	Dispatcher.hpp
	Event.hpp
	EventLoop.hpp
	EventObserver.hpp
	Task.hpp
	TaskQueue.hpp
)

set(processing_sources
	Dispatcher.cpp
	Event.cpp
	EventObserver.cpp
	Task.cpp
	TaskQueue.cpp
)

add_library(Processing STATIC
    ${processing_headers}
    ${processing_sources} 
)

set_target_properties(Processing PROPERTIES LINKER_LANGUAGE CXX CMAKE_CXX_STANDARD 17)