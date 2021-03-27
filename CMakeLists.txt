cmake_minimum_required(VERSION 3.5)

project(physics-engine LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(physics-engine
    albody.h
    alcollision.h
    almath.h
    alworld.h
    alrenderer.h
    alsettings.h
    albody.cpp
    alcollision.cpp
    almath.cpp
    alrenderer.cpp
    alworld.cpp
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    "source/collision/algorithm/sat.cpp"
    "source/collision/algorithm/gjk.cpp"
    "source/collision/algorithm/mpr.cpp"
    "source/collision/algorithm/vclip.cpp"
    "source/collision/collider.cpp"
    "source/collision/detector.cpp"
    "source/collision/contact.cpp"
    "source/common/common.cpp"
    "source/dynamics/rigidbody.cpp"
    "source/dynamics/shape.cpp"
    "source/dynamics/world.cpp"
    "source/math/integrator.cpp"
    "source/math/math.cpp"
    "source/profile/profiler.cpp"
    "source/render/renderer.cpp"
    "include/collision/algorithm/gjk.h"
    "include/collision/algorithm/sat.h"
    "include/collision/algorithm/mpr.h"
    "include/collision/algorithm/vclip.h"
    "include/collision/collider.h"
    "include/collision/contact.h"
    "include/collision/detector.h"
    "include/common/common.h"
    "include/dynamics/rigidbody.h"
    "include/dynamics/shape.h"
    "include/dynamics/world.h"
    "include/math/integrator.h"
    "include/math/math.h"
    "include/profile/profiler.h"
    "include/render/renderer.h"
    )
target_link_libraries(physics-engine PRIVATE Qt5::Widgets)
