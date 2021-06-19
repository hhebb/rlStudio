cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(QT_USE_QTOPENGL TRUE)
# cmake_policy(SET CMP0072 NEW)

find_package(QT NAMES Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets OpenGL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

set(PROJECT_SOURCES
        main.cpp
        window/TestWindow.cpp
        apps/TestApp.cpp
        ../src/physics/World.cpp
        ../src/physics/Body.cpp
        # ../src/physics/TestThread.cpp
        ../src/physics/Collider.cpp
        ../src/physics/Collision.cpp


        # _gltest/RectangleWindow.cpp
        # _gltest/TestDialog.cpp
        # src/OpenGLWindow.cpp
        # src/TriangleWindow.cpp
)


add_executable(mytest ${PROJECT_SOURCES})
# add_dependencies(mytest OpenGL::OpenGL)

target_link_libraries(mytest Qt5::Widgets Qt5::OpenGL ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
target_link_libraries(mytest GL)
target_link_libraries(mytest GLU)
target_link_libraries(mytest glut)