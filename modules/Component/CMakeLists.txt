
cmake_minimum_required(VERSION 3.0)

project(LuComponent)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

set(CMAKE_C_FLAGS " ${CMAKE_C_FLAGS} -std=c99")

#set(CMAKE_CXX_COMPILER "g++")

set(CMAKE_CXX_STANDARD 11)

# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src LuComponent_SRC)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/inc LuComponent_SRC)
set(PROJECT_SOURCES 
    ${LuComponent_SRC}
)

add_library(${PROJECT_NAME} STATIC ${PROJECT_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
