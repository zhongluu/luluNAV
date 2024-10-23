
cmake_minimum_required(VERSION 3.0)

project(LuParseIMU)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

set(CMAKE_C_FLAGS " ${CMAKE_C_FLAGS} -std=c99")

#set(CMAKE_CXX_COMPILER "g++")

set(CMAKE_CXX_STANDARD 11)

# find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src LuParseIMU_SRC)

# add_subdirectory(../Component)
# add_subdirectory(${CMAKE_SOURCE_DIR}/modules/Component ${CMAKE_BINARY_DIR}/modules/Component)

set(PROJECT_SOURCES 
    ${LuParseIMU_SRC}
)

add_library(${PROJECT_NAME} STATIC ${PROJECT_SOURCES})

# target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)

target_link_libraries(${PROJECT_NAME} PUBLIC LuDriver)

target_link_libraries(${PROJECT_NAME} PUBLIC LuComponent)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
