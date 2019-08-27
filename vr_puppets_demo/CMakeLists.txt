cmake_minimum_required(VERSION 2.8.3)
project( vr_puppets_demo)

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
  qt5_wrap_cpp(vr_puppets_demo_MOCS include/vr_puppets_demo/vr_puppets_demo.hpp qcustomplot/qcustomplot.h)
  qt5_wrap_ui(vr_puppets_demo_UIS_H src/vr_puppets_demo.ui)
else()
  qt4_wrap_cpp(vr_puppets_demo_MOCS include/vr_puppets_demo/vr_puppets_demo.hpp qcustomplot/qcustomplot.h)
  qt4_wrap_ui(vr_puppets_demo_UIS_H src/vr_puppets_demo.ui)
endif()

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

add_library( vr_puppets_demo src/vr_puppets_demo.cpp ${vr_puppets_demo_MOCS} ${vr_puppets_demo_UIS_H}
        qcustomplot/qcustomplot.cpp src/vr_puppets_demo.cpp include/vr_puppets_demo/vr_puppets_demo.hpp)

add_dependencies(vr_puppets_demo roboy_middleware_msgs_gencpp)

target_link_libraries(vr_puppets_demo ${catkin_LIBRARIES} )

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(vr_puppets_demo Qt5::Widgets)
else()
  target_link_libraries(vr_puppets_demo ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

#############
## Install ##
#############

install(TARGETS vr_puppets_demo
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES
        package.xml
        plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )