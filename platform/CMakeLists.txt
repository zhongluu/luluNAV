
cmake_minimum_required(VERSION 3.0)

project(LuMiddleware)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

set(CMAKE_C_FLAGS " ${CMAKE_C_FLAGS} -std=c99")

#set(CMAKE_CXX_COMPILER "g++")

set(CMAKE_CXX_STANDARD 11)

include_directories(${CMAKE_SOURCE_DIR}/inc)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src LuMiddleware_SRC)

set(PROJECT_SOURCES 
    ${LuMiddleware_SRC}
)

# 生成静态库，并指定输出目录
add_library(LuMiddleware STATIC ${PROJECT_SOURCES})

find_package(Threads REQUIRED)
target_link_libraries(LuMiddleware PRIVATE Threads::Threads)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
