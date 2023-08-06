cmake_minimum_required(VERSION 3.10)

project(myPGO)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)

message(STATUS "Found Ceres: ${CERES_FOUND}")
message(STATUS "Ceres Include Dirs: ${CERES_INCLUDE_DIRS}")
message(STATUS "Ceres Libs: ${CERES_LIBRARIES}")

message(STATUS "Found Sophus: ${Sophus_FOUND}")
message(STATUS "Sophus Include Dirs: ${Sophus_INCLUDE_DIRS}")
message(STATUS "Sophus Libs: ${Sophus_LIBRARIES}")

message(STATUS "Found Eigen: ${Eigen3_FOUND}")
message(STATUS "Eigen Include Dirs: ${Eigen_INCLUDE_DIRS}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                                        ${CERES_INCLUDE_DIRS} 
                                        ${SOPHUS_INCLUDE_DIRS}
                                        ${EIGEN3_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/main.cc src/optimizer.cc 
                                    src/ceres_definition.cc)

target_link_libraries(${PROJECT_NAME} 
                                            ${CERES_LIBRARIES} 
                                            ${SOPHUS_LIBRARIES})

