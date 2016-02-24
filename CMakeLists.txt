cmake_minimum_required(VERSION 2.8.3)
project(eye_in_hand)

find_package( OpenCV 2.4 REQUIRED)

# find_package( OpenCV 3.0.0 REQUIRED )
# if( OpenCV_FOUND )
# list( APPEND ThirdParty_LIBS ${OpenCV_LIBS} )
#     include_directories( opencv_contrib ${OpenCV_INCLUDE_DIRS} )
# endif( OpenCV_FOUND )

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/daniela/OpenCV3/opencv_contrib) 



# include_directories(cmake_modules ${Boost_INCLUDE_DIR})
find_package(Eigen3 REQUIRED COMPONENTS Dense Geometry)


if ( EIGEN3_FOUND )
  INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})
else()
    message(STATUS "This program requires the Eigen Headers, and will not be compiled.")
endif()


# include_directories(include ${Boost_INCLUDE_DIR})

# <!-- find_package(orocos_kdl REQUIRED) -->
find_package(catkin REQUIRED 
  sensor_msgs
  controller_interface
  control_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  urdf
  roscpp
  kdl_parser
  message_generation
  cmake_modules
  tf_conversions 
  message_generation 
  std_msgs
  geometry_msgs 
  sensor_msgs
  image_transport
  cv_bridge
)

# <!-- find_package(PCL 1.7.2 REQUIRED) -->

# <!-- include_directories(${GAZEBO_INCLUDE_DIRS})
# link_directories(${GAZEBO_LIBRARY_DIRS})
# include_directories(include ${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS}) -->

link_directories(${catkin_LIBRARY_DIRS})
# <!-- link_directories(${orocos_kdl_LIBRARY_DIRS}) -->


add_definitions(-std=c++11 --all-warnings -O3) 

# <!-- find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS}) -->
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
# include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )
# add a target to generate API documentation with Doxygen


# find_package(Doxygen)
# if(DOXYGEN_FOUND)
#   configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.in @ONLY)
#   add_custom_target(doc
#   ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/doxygen.in
#   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#   COMMENT "Generating API documentation with Doxygen" VERBATIM)
# endif(DOXYGEN_FOUND)


# <!-- add_message_files(
#   FILES

# ) -->

# add_service_files(
#   FILES
# ObstacleInfo.srv
# ObstacleInfoArray.srv
# )


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
 )

catkin_package(
  DEPENDS orocos_kdl
  CATKIN_DEPENDS
  controller_interface
  control_msgs
  control_toolbox
  realtime_tools
  forward_command_controller
  message_runtime
  tf_conversions
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})


# add_executable(testa src/test_whit_ptam_and_ros.cpp)
# # add_executable(testa src/image_converter.cpp)
# add_dependencies(testa testa_gencpp ${PROJECT_NAME})
# target_link_libraries(testa ${catkin_LIBRARIES} ${OpenCV_LIBS} opencv_nonfree)


add_executable(prova_dany src/main_image_.cpp)
# add_executable(prova_dany src/image_converter.cpp)
add_dependencies(prova_dany prova_dany_gencpp ${PROJECT_NAME})
target_link_libraries(prova_dany ${catkin_LIBRARIES} ${OpenCV_LIBS} opencv_nonfree)

add_executable(ptam_test_dany src/dany_test.cpp)
# add_executable(prova_dany src/image_converter.cpp)
add_dependencies(ptam_test_dany ptam_test_dany_gencpp ${PROJECT_NAME})
target_link_libraries(ptam_test_dany ${catkin_LIBRARIES} ${OpenCV_LIBS} opencv_nonfree)



install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})