cmake_minimum_required(VERSION 3.21)

find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(SOURCES
#        src/DataAnalysis.cpp
#        src/RobotMath.cpp
#        src/TrajectoryGenerator/CubicTrajectoryGenerator.cpp
#        src/TrajectoryGenerator/QuadTrajectoryGenerator.cpp
#        src/TrajectoryGenerator/QuinticTrajectoryGenerator.cpp
#        src/TrajectoryGenerator/SineTrajectoryGenerator.cpp
        src/Filter.cpp
#        src/FloatingBaseModel.cpp
)

ADD_COMPILE_DEFINITIONS (LOG_DIR="${CMAKE_SOURCE_DIR}/log/")

message(STATUS "${ColorGreen}CAMEL SDK VERSION 1.16.1${ColorReset}")

add_library(CAMEL_SDK STATIC
        ${SOURCES}
)

target_include_directories(CAMEL_SDK
        PUBLIC
        include
)

target_link_libraries(CAMEL_SDK
        Eigen3::Eigen
#        pthread
#        Quad_Parameters
#        ${RBDL_LIBRARY}
)

target_compile_definitions(CAMEL_SDK INTERFACE SIMUL=true)