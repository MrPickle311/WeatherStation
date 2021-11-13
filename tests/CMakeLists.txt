cmake_minimum_required(VERSION 3.5)

project(Tests LANGUAGES CXX)

add_definitions(-DGTEST_LANGUAGE_CXX11)
set(GTEST_COLOR=1)

set(GOOGLETEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../extern/googletest")

if (EXISTS ${GOOGLETEST_DIR})
    set(GTestSrc ${GOOGLETEST_DIR}/googletest)
    set(GMockSrc ${GOOGLETEST_DIR}/googlemock)
	message("Found googletest in project folder")
	#set(CMAKE_PREFIX_PATH 
	#find_package(GTest REQUIRED PATHS "${GOOGLETEST_DIR}/build/lib/libgtest.a" ) 
	find_package(GTest)
elseif (UNIX AND EXISTS /usr/src/gtest)
    set(GTestSrc /usr/src/gtest)
    message(WARNING "Using gtest from system")
    if (EXISTS /usr/src/gmock)
        set(GMockSrc /usr/src/gmock)
		find_package(GTest)
    endif ()
else ()
    message( FATAL_ERROR "No googletest src dir found - set GOOGLETEST_DIR to enable!")
endif ()

set(GTestFiles ${GTestSrc}/src/gtest-all.cc)
set(GTestIncludes ${GTestSrc} ${GTestSrc}/include)

if (NOT ${GMockSrc} STREQUAL "")
    list(APPEND GTestFiles ${GMockSrc}/src/gmock-all.cc)
    list(APPEND GTestIncludes ${GMockSrc} ${GMockSrc}/include)
endif ()

set(LIBS
	Communication
	Processing
	Engine
	Sensors
)

set(TESTED_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/Processing/Dispatcher.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/Processing/Event.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/Processing/EventLoop.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/Processing/EventObserver.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/Processing/TaskQueue.hpp
	
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/Devices/Sensor/LPS22.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/../src/Devices/Sensor/HTS22.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/../src/Devices/Sensor/I2CSensor.hpp
	
	${CMAKE_CURRENT_SOURCE_DIR}/../src/Devices/Engine/Engines.hpp
	
	${CMAKE_CURRENT_SOURCE_DIR}/../src/Devices/Communication/BluetoothPort.hpp
)

enable_testing()

function(add_test name)
    add_executable(${name} ${name}.hpp ${name}.cpp ${GTestFiles} ${TESTED_HEADERS})
    target_include_directories(${name} PUBLIC ${INCLUDES} ${GTestIncludes})
    #target_compile_definitions(${name}
    #   PRIVATE $<$<OR: $<CONFIG:Debug> , $<CONFIG:RelWithDebInfo> >:QT_QML_DEBUG>
	#)
    gtest_discover_tests(${name})
    target_link_libraries(${name} PRIVATE
       Threads::Threads
       ${LIBS})
endfunction()



add_test(EventLoopTests) 