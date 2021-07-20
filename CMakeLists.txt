project(rom_nav)

cmake_minimum_required (VERSION 2.8) 
#cmake_policy(SET CMP0015 NEW)


# 编译方式
#set(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )


message(STATUS "This is SOURCE dir :" ${PROJECT_SOURCE_DIR})
message(STATUS "This is BINARY dir :" ${PROJECT_BINARY_DIR})
message(STATUS "This is PROJECT_NAME:" ${PROJECT_NAME})
message(STATUS "This is OpenCV_LIBS:" ${OpenCV_LIBS})
message(STATUS "Before This is CMAKE_BUILD_TYPE :" ${CMAKE_BUILD_TYPE})
message(STATUS "This is PROJECT_SOURCE_DIR:" ${PROJECT_SOURCE_DIR})

if ("${CMAKE_BUILD_TYPE}" STREQUAL "") # STREQUAL:相同
  #set(CMAKE_BUILD_TYPE "Release")
  set(CMAKE_BUILD_TYPE "Debug")
endif()

message(STATUS "After This is CMAKE_BUILD_TYPE :" ${CMAKE_BUILD_TYPE})


if(NOT DEFINED BUILD_ROM_NAV_EXAMPLES)
  set(BUILD_ROM_NAV_EXAMPLES "ON")
endif(NOT DEFINED BUILD_ROM_NAV_EXAMPLES)

message(STATUS "After This is BUILD_ROM_NAV_EXAMPLES :" ${BUILD_ROM_NAV_EXAMPLES})

#find_package(Boost REQUIRED)  
find_package(Eigen3 REQUIRED)
find_package(Threads)
#find_package(Ceres REQUIRED)
#find_package(OpenCV REQUIRED )

include_directories("/usr/include/eigen3")

include_directories(./include
                    /usr/include
                    #${Boost_INCLUDE_DIRS}
                    #${OpenCV_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIR}
                    #${CERES_INCLUDE_DIRS}
		   )


#Vis3D
#find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED )
#find_package(OpenGL REQUIRED)
#find_package(GLUT REQUIRED)

message(STATUS "This is OPENGL_INCLUDE_DIRS :" ${OPENGL_INCLUDE_DIRS})
message(STATUS "This is GLUT_INCLUDE_DIRS:" ${GLUT_INCLUDE_DIRS})
message(STATUS "This is CMAKE_CURRENT_BINARY_DIR:" ${CMAKE_CURRENT_BINARY_DIR})

include_directories( ${OPENGL_INCLUDE_DIRS}  
                     ${GLUT_INCLUDE_DIRS}
                     ${CMAKE_CURRENT_BINARY_DIR} )


# 变量后面用到                     
#set(LIBS_SOURCES  )  #后面是文件名路径
#set(LIBS_HEADERS )   #后面是文件名路径
aux_source_directory( ./src/lib LIBS_SOURCES ) # 该命令会查找指定目录下的所有源文件，然后将结果存进指定变量名
aux_source_directory( ./include/lib LIBS_HEADERS ) # 该命令会查找指定目录下的所有源文件，然后将结果存进指定变量名
aux_source_directory( ./src/sensor SENSOR_SOURCES ) # 该命令会查找指定目录下的所有源文件，然后将结果存进指定变量名
aux_source_directory( ./include/sensor SENSOR_HEADERS ) # 该命令会查找指定目录下的所有源文件，然后将结果存进指定变量名
aux_source_directory( ./src/udp UDP_SOURCES ) # 该命令会查找指定目录下的所有源文件，然后将结果存进指定变量名
aux_source_directory( ./include/udp UDP_HEADERS ) # 该命令会查找指定目录下的所有源文件，然后将结果存进指定变量名
aux_source_directory( ./src/serial SERIAL_SOURCES ) # 该命令会查找指定目录下的所有源文件，然后将结果存进指定变量名
aux_source_directory( ./include/serial SERIAL_HEADERS ) # 该命令会查找指定目录下的所有源文件，然后将结果存进指定变量名
aux_source_directory( ./src/nav NAV_SOURCES ) # 该命令会查找指定目录下的所有源文件，然后将结果存进指定变量名
aux_source_directory( ./include/nav NAV_HEADERS ) # 该命令会查找指定目录下的所有源文件，然后将结果存进指定变量名


#include(${QT_USE_FILE})
#message(STATUS "This is QT_USE_FILE:" ${QT_USE_FILE})
#add_definitions(${QT_DEFINITIONS})  # 宏定义

aux_source_directory( ./src SINS_SRC ) # 该命令会查找指定目录下的所有源文件，然后将结果存进指定变量名
set( ROM_NAV_SRC ${SINS_SRC} ${LIBS_SOURCES} ${SENSOR_SOURCES} ${UDP_SOURCES} ${NAV_SOURCES} ${SERIAL_SOURCES})

add_library(rom_nav ${ROM_NAV_SRC})
# set_target_properties(rom_nav PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib) # 编译静库 动态库 指定输出路径 lib

set (ROM_NAV_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE STRING "hybrid_sins include directories")
set (ROM_NAV_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib CACHE STRING "hybrid_sins libraries directories")
set (ROM_NAV_LIBS rom_nav ${CERES_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT}
     CACHE STRING "rom_nav libraries")

message( STATUS "This is ROM_NAV_LIBS :" "${ROM_NAV_LIBS}" )
#message( "${ROM_NAV_LIBS}" )

if( BUILD_ROM_NAV_EXAMPLES )

add_executable(navigation apps/navigation.cpp)
target_link_libraries( navigation ${ROM_NAV_LIBS})
#set_target_properties( navigation PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(sim_navigation apps/sim_navigation.cpp)
target_link_libraries( sim_navigation ${ROM_NAV_LIBS})
#set_target_properties( sim_navigation PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(sanchi_SINS_serial apps/sanchi_SINS_serial.cpp)
target_link_libraries( sanchi_SINS_serial ${ROM_NAV_LIBS})
#set_target_properties( sanchi_SINS_serial PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(sim_control apps/sim_control.cpp)
target_link_libraries( sim_control ${ROM_NAV_LIBS})
#set_target_properties( sim_control PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(simple_test apps/simple_test.cpp)
target_link_libraries( simple_test ${ROM_NAV_LIBS})
#set_target_properties( simple_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

endif( BUILD_ROM_NAV_EXAMPLES )

