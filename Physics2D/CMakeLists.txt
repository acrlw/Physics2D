cmake_minimum_required(VERSION 3.6)

project(Physics2D LANGUAGES CXX)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(Physics2D
    SHARED
    "source/collision/algorithm/sat.cpp"
    "source/collision/algorithm/gjk.cpp"
    "source/collision/algorithm/mpr.cpp"
    "source/collision/algorithm/clip.cpp"
    "source/collision/broadphase/dbvh.cpp"
    "source/collision/broadphase/aabb.cpp"
    "source/collision/continuous/ccd.cpp" 
    "source/collision/collider.cpp"
    "source/collision/detector.cpp"
    "source/collision/broadphase/grid.cpp" 
    "source/collision/broadphase/tree.cpp" 
    "source/collision/algorithm/simplex.cpp" 
    "source/common/common.cpp"
    "source/dynamics/body.cpp"
    "source/dynamics/world.cpp"
    "source/math/integrator.cpp"
    "source/math/math.cpp"
    "source/geometry/algorithm/2d.cpp"
    "source/math/linear/vector2.cpp"
    "source/math/linear/vector3.cpp"
    "source/math/linear/matrix2x2.cpp"
    "source/math/linear/matrix3x3.cpp"
    "source/utils/random.cpp" 
    "source/dynamics/constraint/contact.cpp" 
    "source/geometry/shape/point.cpp"
    "source/geometry/shape/polygon.cpp"
    "source/geometry/shape/rectangle.cpp"
    "source/geometry/shape/circle.cpp"
    "source/geometry/shape/ellipse.cpp"
    "source/geometry/shape/edge.cpp"
    "source/geometry/shape/curve.cpp"
    "source/geometry/shape/capsule.cpp"
    "source/geometry/shape/sector.cpp"
)