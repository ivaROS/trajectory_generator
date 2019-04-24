cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

# LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp
#                                         std_msgs
#                                         nav_msgs                                              
#                                         geometry_msgs
#                                         tf
                                        )



## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}  #Without the Eigen3 directories, the package doesn't compile properly with catkin build
#   LIBRARIES 
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs tf
#   DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include
)

add_compile_options(-std=c++11 -Wall)


## Declare a cpp executable
add_executable(odeint_demo src/odeint_demo.cpp )
add_executable(odeint_basic src/odeint_basic.cpp )
add_executable(odeint_observer src/odeint_observer.cpp )
# add_executable(near_identity_test src/near_identity_test.cpp )
# add_executable(odeint_controller src/odeint_controller.cpp )
# add_library(traj_generator src/traj_generator.cpp )
# add_executable(test_traj_generator src/test_traj_generator.cpp )
# add_library(trajectory_generator_ros_interface src/trajectory_generator_ros_interface.cpp )
# add_executable(test_ros_interface src/test_ros_interface.cpp )
# 
# add_dependencies(trajectory_generator_ros_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 
# ## Add cmake target dependencies of the executable/library
# ## as an example, message headers may need to be generated before nodes
# # add_dependencies(trajectory_generator_node trajectory_generator_generate_messages_cpp)
# 
## Specify libraries to link a library or executable target against
target_link_libraries(odeint_demo
   ${catkin_LIBRARIES}
 )
 
target_link_libraries(odeint_basic
   ${catkin_LIBRARIES}
 )
 
target_link_libraries(odeint_observer
   ${catkin_LIBRARIES}
 )
#  
#  target_link_libraries(odeint_controller
#    ${catkin_LIBRARIES}
#      ${EIGEN3_LIBS}
#  )
#  
#  target_link_libraries(traj_generator
#    ${catkin_LIBRARIES}
#      ${EIGEN3_LIBS}
#  )
#  
#   target_link_libraries(test_traj_generator
#    ${catkin_LIBRARIES}
#      ${EIGEN3_LIBS}
#  )
#  
#   target_link_libraries(trajectory_generator_ros_interface
#    ${catkin_LIBRARIES}
#      ${EIGEN3_LIBS}
#     traj_generator
#  )
# 
#   target_link_libraries(test_ros_interface
#    ${catkin_LIBRARIES}
#  )

 
#  add_executable(odeint_custom_container src/odeint_custom_container.cpp )
# 
#   target_link_libraries(odeint_custom_container
#    ${catkin_LIBRARIES}
#      ${EIGEN3_LIBS}
# )
