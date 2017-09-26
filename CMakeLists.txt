cmake_minimum_required(VERSION 2.8.9)
project(osa_gui)
find_package(catkin REQUIRED COMPONENTS roscpp osa_msgs osa_control)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5PrintSupport REQUIRED)

find_package(Qt5OpenGL REQUIRED)
find_package(OpenGL)

SET(CMAKE_AUTOMOC ON)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

add_compile_options(-std=c++11)

catkin_package(
  INCLUDE_DIRS include/common include/gui include/gui/qcustomplot include/rosnode
  CATKIN_DEPENDS roscpp osa_msgs osa_control
  DEPENDS system-lib
)

include_directories(include/common include/common/osa_msgs_json include/gui include/gui/qcustomplot include/rosnode include/sequencer)
include_directories(${catkin_INCLUDE_DIRS})

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/common/*.h include/common/osa_msgs_json/*.h include/gui/*.h include/gui/qcustomplot/*.h include/rosnode/*.h include/sequencer/*.h)
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/main.cpp src/common/*.cpp src/common/osa_msgs_json/*.cpp src/gui/*.cpp src/gui/qcustomplot/*.cpp src/rosnode/*.cpp src/sequencer/*.cpp)

qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt5_wrap_cpp(QT_MOC_HPP ${QT_MOC})

include_directories(src ${catkin_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})

add_executable(osa_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
qt5_use_modules(osa_gui Core Gui Widgets OpenGL)				   
target_link_libraries(osa_gui ${QT_LIBRARIES} ${catkin_LIBRARIES} GL Qt5::Widgets Qt5::PrintSupport ${Qt5Gui_EGL_LIBRARIES} ${Qt5Gui_OPENGL_LIBRARIES} ${OPENGL_LIBRARIES})

install(TARGETS osa_gui
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

