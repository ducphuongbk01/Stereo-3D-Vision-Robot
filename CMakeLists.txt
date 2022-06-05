cmake_minimum_required(VERSION 3.5)

project(ThesisGUI VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check https://doc.qt.io/qt/deployment-android.html for more information.
# They need to be set before the find_package( ...) calls below.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)
find_package(Qt5 COMPONENTS Network REQUIRED)
find_package(Qt5 COMPONENTS SerialPort REQUIRED)

find_package( OpenCV REQUIRED )
set(DEPENDENCIES realsense2 ${OpenCV_LIBS} )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization filters)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

find_package(VISP REQUIRED)
if(VISP_FOUND)
    include(${VISP_USE_FILE})
endif(VISP_FOUND)
include_directories(${SOURCE_DIR} ${VISP_INCLUDE_DIRS})

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui

        calibcolor.cpp
        calibcolor.h
        calibcolor.ui

        calibpose.cpp
        calibpose.h
        calibpose.ui

        controllib.cpp
        controllib.h

        convert.cpp
        convert.h

        serialport.cpp
        serialport.h

        udp.cpp
        udp.h

        calibration.cpp
        calibration.h

        ObjectDetection.cpp
        ObjectDetection.h

        PointCloudCreator.cpp
        PointCloudCreator.h

        realsensecamera.cpp
        realsensecamera.h

        poseestimation.cpp
        poseestimation.h

        process.cpp
        process.h

        evaluatepose.cpp
        evaluatepose.h
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(ThesisGUI
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
    if(ANDROID)
        add_library(ThesisGUI SHARED
            ${PROJECT_SOURCES}
        )
    else()
        add_executable(ThesisGUI
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(ThesisGUI PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(ThesisGUI PRIVATE Qt5::Network)
target_link_libraries(ThesisGUI PRIVATE Qt5::SerialPort)
target_link_libraries(ThesisGUI PRIVATE ${DEPENDENCIES})
target_link_libraries(ThesisGUI PRIVATE ${VISP_LIBRARIES})
target_link_libraries(ThesisGUI PRIVATE ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}
                                        ${PCL_LIBRARIES} ${PCL_FILTERS_LIBRARIES} pcl_visualization)
target_link_libraries(ThesisGUI PRIVATE boost_thread boost_system)

set_target_properties(ThesisGUI PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(ThesisGUI)
endif()
