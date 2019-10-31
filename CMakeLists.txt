cmake_minimum_required(VERSION 2.8)
project(instanceAnnotation) # Your project name

set(CMAKE_CXX_STANDARD 11) # This is equal to QMAKE_CXX_FLAGS += -std=c++0x

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
# Instruct CMake to run uic automatically when needed.
set(CMAKE_AUTOUIC ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")    # O3 改 O0作为Debug
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# This will find the Qt5 files. You will need a QT5_DIR env variable
find_package(Qt5Widgets REQUIRED) # Equivalent of QT += widgets

set(SOURCES
    main.cpp mainwindow.cpp utility.cpp instance.cpp
    visualize.cpp
    autodataassociation.cpp)
set(UI mainwindow.ui)


# For opencv
set( OpenCV_DIR "/opt/ros/kinetic/share/OpenCV-3.3.1-dev/" )
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

# For ellipsoid_slam core
include_directories(
    /home/jhz/workspace/projects/ellipsoid-slam/core/include
    /home/jhz/workspace/projects/ellipsoid-slam/core
    )

# For PCL
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# This will create you executable
add_executable(${PROJECT_NAME} ${SOURCES} ${UI})
# This will link necessary Qt5 libraries to your project
target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    )
target_link_libraries(${PROJECT_NAME}
    /home/jhz/workspace/projects/ellipsoid-slam/core/lib/libutils.so
    /home/jhz/workspace/projects/ellipsoid-slam/core/lib/libEllipsoidSLAM.so
    /home/jhz/workspace/projects/ellipsoid-slam/core/lib/libtum_rgbd.so
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    boost_system
    )
