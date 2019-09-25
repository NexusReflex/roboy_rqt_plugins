cmake_minimum_required(VERSION 2.8.3)
project(vr_puppets)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")

find_package(catkin REQUIRED COMPONENTS roscpp roslint rqt_gui rqt_gui_cpp std_msgs roboy_middleware_msgs common_utilities)
catkin_package(CATKIN_DEPENDS roboy_middleware_msgs)

include_directories( include qcustomplot)

## System dependencies are found with CMake's conventions

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

roslint_cpp()

catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs roboy_middleware_msgs)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(vr_puppets_MOCS include/vr_puppets/vr_puppets.hpp qcustomplot/qcustomplot.h)
  qt5_wrap_ui(vr_puppets_UIS_H src/vr_puppets.ui)
else()
  qt4_wrap_cpp(vr_puppets_MOCS include/vr_puppets/vr_puppets.hpp qcustomplot/qcustomplot.h)
  qt4_wrap_ui(vr_puppets_UIS_H src/vr_puppets.ui)
endif()

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME} src/vr_puppets.cpp ${vr_puppets_MOCS} ${vr_puppets_UIS_H}
        qcustomplot/qcustomplot.cpp src/vr_puppets.cpp include/vr_puppets/vr_puppets.hpp)

add_dependencies(${PROJECT_NAME} roboy_middleware_msgs_gencpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES
        package.xml
        plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )