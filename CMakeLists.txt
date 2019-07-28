cmake_minimum_required(VERSION 2.8.3)
project(fira_esi)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        mavros
        roscpp
        std_msgs
        dynamic_reconfigure
        cv_bridge
        image_transport
        message_generation
        tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(ZED 2 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
        cfg/fira_esi_dynamic_cfg.cfg
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## CATKIN_DEPENDS: catkin_packages dependent projects also need
catkin_package(
        CATKIN_DEPENDS geometry_msgs mavros roscpp std_msgs dynamic_reconfigure cv_bridge image_transport
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${ZED_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        include
        ../../devel/include
        src
)

link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})



## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(fira_esi_node
        src/fira_esi_node.cpp

        src/interface/DynamicRecfgInterface.h
        src/interface/DynamicRecfgInterface.cpp
        src/interface/PX4Interface.h
        src/interface/PX4Interface.cpp
        src/interface/VisionInterface.h
        src/interface/VisionInterface.cpp

        src/controller/Action.h
        src/controller/Action.cpp
        src/controller/Task.h
        src/controller/Task.cpp
        src/controller/FlowController.h
        src/controller/FlowController.cpp
        src/controller/PIDController.h
        src/controller/PIDController.cpp

        src/vision/alldefine.h
        src/vision/findlines.h
        src/vision/findlines.cpp
        src/vision/linezxz.h
        src/vision/linezxz.cpp
        src/vision/mycv.h
        src/vision/mycv.cpp

        src/utils/utils.h
        src/utils/utils.cpp
        src/utils/JudgeAchieveCounter.cpp
        src/utils/JudgeAchieveCounter.h)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(fira_esi_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(fira_esi_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        yaml-cpp
        ${ZBAR_LIBS}
        zbar
        ${ZED_LIBS}
)


