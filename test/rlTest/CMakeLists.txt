# test build targets
cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(QT_USE_QTOPENGL TRUE)
# cmake_policy(SET CMP0072 NEW)

# find library.
find_package(QT NAMES Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets OpenGL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(jsoncpp REQUIRED)

# set sources for target.
set(PHYSICS_SOURCES
        main.cpp
        window/TestWindow.cpp
        apps/TestApp.cpp
        ../src/physics/World.cpp
        ../src/physics/Body.cpp
        # ../src/physics/TestThread.cpp
        ../src/physics/Collider.cpp
        ../src/physics/Collision.cpp
        ../src/Helper.cpp
        ../src/physics/Joint.cpp
        ../src/physics/DistanceJoint.cpp
        ../src/physics/RevoluteJoint.cpp
        ../src/physics/PrismaticJoint.cpp
        ../src/physics/WorldParser.cpp

        # _gltest/RectangleWindow.cpp
        # _gltest/TestDialog.cpp
        # src/OpenGLWindow.cpp
        # src/TriangleWindow.cpp
)

set(RL_SOURCES
        main.cpp
        TestApp.cpp
        window/TestWindow.cpp
        ../../src/rl/Agent.cpp
        ../../src/rl/Environment.cpp
        ../../src/rl/Runner.cpp
)

# add build target.
add_executable(rl_test ${RL_SOURCES})

# link founded libraries.
target_link_libraries(rl_test Qt5::Widgets Qt5::OpenGL ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
target_link_libraries(rl_test GL)
target_link_libraries(rl_test GLU)
target_link_libraries(rl_test glut)
target_link_libraries(rl_test jsoncpp)