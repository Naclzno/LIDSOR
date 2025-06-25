# ID: Naclzno
# Name: Xinyuan Yan
# Email: yan1075783878@gmail.com

cmake_minimum_required(VERSION 3.12.0)
project(LIDSOR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON) 
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin) 

find_package(PCL REQUIRED)
find_package(gflags REQUIRED)

set(CODE_SOURCE_DIR ${CMAKE_SOURCE_DIR}/include) 
set(SRC_SOURCE_DIR ${CMAKE_SOURCE_DIR}/src) 

add_definitions(${PCL_DEFINITIONS})
include_directories(${CODE_SOURCE_DIR})

# LIDSOR 
set(LIDSOR lidsor)
add_executable(${LIDSOR}
  ${SRC_SOURCE_DIR}/filters/lidsor.cc
)
target_include_directories(${LIDSOR} PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${LIDSOR}
  ${PCL_LIBRARIES}
  ${GFLAGS_LIBRARIES}
)

# EVA 
set(EVA eva)
add_executable(${EVA}
  ${SRC_SOURCE_DIR}/eva.cc
)
target_include_directories(${EVA} PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${EVA}
  ${PCL_LIBRARIES}
  ${GFLAGS_LIBRARIES}
)

# Timer 
set(Timer timer)
add_executable(${Timer}
  ${SRC_SOURCE_DIR}/timer.cc
)
