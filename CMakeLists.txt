cmake_minimum_required(VERSION 3.15)
project(kite_energy_system)

list(APPEND CMAKE_MODULE_PATH "$ENV{CMAKE_MODULE_DIR}" "$ENV{CMAKE_MODULE_DIR}/Eigen3")


set(CMAKE_CXX_STANDARD 11)
include(GNUInstallDirs)

set(CMAKE_CXX_FLAGS "-O3")

find_package(Boost 1.71.0)
find_package(Eigen3 REQUIRED)
find_package(glog 0.4.0 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

if(Boost_FOUND)

    add_subdirectory(kite_dynamics)    

    #include_directories(${Boost_INCLUDE_DIRS})

    #add_executable(lorenzSystem lorenzSystem.cc)

    #target_link_libraries(lorenzSystem ${Boost_LIBRARIES})

    #add_executable(point_lorenz point_lorenz.cc)

    #target_link_libraries(point_lorenz ${Boost_LIBRARIES})

endif()

#add_executable(EigenTest EigenTest.cc)

#add_executable(glogTest glogTest.cc)
#target_link_libraries(glogTest glog::glog)