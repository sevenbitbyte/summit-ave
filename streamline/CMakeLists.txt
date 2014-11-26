cmake_minimum_required(VERSION 2.8.8)

project(streamline)

#cmake_minimum_required(VERSION 2.8)
if(CMAKE_BUILD_TYPE STREQUAL Release)
  SET(CMAKE_BUILD_TYPE Release)
  SET (PROJECT_NAME streamline)
  message("Release mode")
else()
  SET(CMAKE_BUILD_TYPE Debug)
  SET (PROJECT_NAME streamline)
  SET(CMAKE_CXX_FILES "-g -Wall")
  message("Debug mode")

endif()

#find QT libraries
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Xml REQUIRED)
find_package(OpenGL REQUIRED)
#find_package(QtSql REQUIRED)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )
# We need add -DQT_WIDGETS_LIB when using QtWidgets in Qt 5.
add_definitions(${Qt5Widgets_DEFINITIONS})


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

FIND_PROGRAM(QT_UIC_EXECUTABLE uic
  $ENV{QTDIR}/bin
  )



QT5_WRAP_UI(UIS_HDRS src/ui/mainwindow.ui)

SET(SRC

	src/main.cpp
	src/contentmanager.cpp
	src/datastore/table.cpp
	src/datastore/database.cpp
	src/datastore/datum.cpp
	src/datastore/types/filerecord.cpp
	src/datastore/types/locationdatum.cpp
	src/datastore/types/locationsegment.cpp
	src/ui/mapwidget.cpp
	src/ui/mainwindow.cpp
	src/gps/UTM/conversion.cpp
	src/gps/trackpoint.cpp

)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${Qt5Widgets_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS}
)

# set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION  )
add_executable(streamline ${SRC} ${UIS_HDRS})
target_link_libraries(streamline ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )

qt5_use_modules(streamline Core Gui Widgets OpenGL Xml Network Sql)
