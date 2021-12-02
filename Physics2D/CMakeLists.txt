cmake_minimum_required(VERSION 3.6)

project(Physics2D LANGUAGES CXX)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES_FILES 
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

set(HEADERS_FILE 
    "include/collision/algorithm/clip.h"
    "include/collision/algorithm/gjk.h"
    "include/collision/algorithm/minkowski.h"
    "include/collision/algorithm/mpr.h"
    "include/collision/algorithm/sat.h"
    "include/collision/algorithm/simplex.h"

    "include/collision/broadphase/aabb.h"
    "include/collision/broadphase/dbvh.h"
    "include/collision/broadphase/grid.h"
    "include/collision/broadphase/tree.h"

    "include/collision/continuous/ccd.h"
    "include/collision/collider"
    "include/collision/detector"

    "include/common/common.h"

    "include/dynamics/constraint/constraint.h"
    "include/dynamics/constraint/contact.h"

    "include/dynamics/joint/distance.h"
    "include/dynamics/joint/joint.h"
    "include/dynamics/joint/joints.h"
    "include/dynamics/joint/point.h"
    "include/dynamics/joint/pulley.h"
    "include/dynamics/joint/revolute.h"
    "include/dynamics/joint/rotation.h"

    "include/dynamics/body.h"
    "include/dynamics/world.h"

    "include/geometry/algorithm/2d.h"

    "include/geometry/shape/capsule.h"
    "include/geometry/shape/circle.h"
    "include/geometry/shape/curve.h"
    "include/geometry/shape/edge.h"
    "include/geometry/shape/ellipse.h"
    "include/geometry/shape/point.h"
    "include/geometry/shape/polygon.h"
    "include/geometry/shape/rectangle.h"
    "include/geometry/shape/sector.h"

    "include/geometry/shape.h"

    "include/math/linear/linear.h"
    "include/math/linear/matrix2x2.h"
    "include/math/linear/matrix3x3.h"
    "include/math/linear/vector2.h"
    "include/math/linear/vector3.h"
    
    "include/math/integrator.h"
    "include/math/math.h"

    "include/utils/random.h"

    "include/physics2d.h"
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(Physics2D STATIC ${SOURCES_FILES} ${HEADERS_FILE})

target_include_directories(Physics2D
                           INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include
                           )