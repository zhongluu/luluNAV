
cmake_minimum_required(VERSION 3.0)

project(LuNavigation)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

set(CMAKE_C_FLAGS " ${CMAKE_C_FLAGS} -std=c99")

#set(CMAKE_CXX_COMPILER "g++")

set(CMAKE_CXX_STANDARD 11)

add_subdirectory(${CMAKE_SOURCE_DIR}/driver)

# add_subdirectory(${CMAKE_SOURCE_DIR}/modules/Component ${CMAKE_BINARY_DIR}/modules/Component)

add_subdirectory(${CMAKE_SOURCE_DIR}/modules/Component)

add_subdirectory(${CMAKE_SOURCE_DIR}/modules/INSCore)

add_subdirectory(${CMAKE_SOURCE_DIR}/modules/ParseIMU)

add_subdirectory(${CMAKE_SOURCE_DIR}/modules/ParseGNSS)

add_subdirectory(${CMAKE_SOURCE_DIR}/platform)

include_directories(${CMAKE_SOURCE_DIR}/inc)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src LuNavigation_SRC)

set(PROJECT_SOURCES 
    ${LuNavigation_SRC}
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

find_package(Eigen3 REQUIRED)

find_package(Threads REQUIRED)

target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads Eigen3::Eigen LuMiddleware LuINSCore LuParseIMU LuParseGNSS LuDriver)
