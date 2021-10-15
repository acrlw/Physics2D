cmake_minimum_required(VERSION 3.5)

project(physics-engine LANGUAGES CXX)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)




add_executable(physics-engine
    main.cpp
    "include/collision/algorithm/gjk.h"
    "include/collision/algorithm/sat.h"
    
    "include/collision/algorithm/clip.h"
    "include/collision/collider.h"
    
    "include/collision/detector.h"
    "include/common/common.h"
    "include/dynamics/body.h"
    "include/geometry/shape.h"
    "include/dynamics/world.h"
    "include/math/integrator.h"
    "include/math/math.h"
    "include/geometry/algorithm/2d.h"
    "include/utils/profiler.h"
    "include/render/renderer.h"
    "include/math/linear/vector2.h"
    "include/math/linear/vector3.h"
    "include/math/linear/matrix2x2.h"
    "include/math/linear/linear.h"
    "include/physics2d.h"
    "tests/test.h"
    "tests/test_math.h"
    "tests/test_integrator.h"
    "tests/test_gjk.h"
    "tests/test_geomentry.h" 
    "testbed/testbed.h" 
    "testbed/testbed.cpp"
    "include/collision/algorithm/mpr.h"
    "source/collision/algorithm/sat.cpp"
    "source/collision/algorithm/gjk.cpp"
    "source/collision/algorithm/mpr.cpp"
    "source/collision/algorithm/clip.cpp"
    "source/collision/collider.cpp"
    "source/collision/detector.cpp"
    
    "source/common/common.cpp"
    "source/dynamics/body.cpp"
    "source/dynamics/world.cpp"
    "source/math/integrator.cpp"
    "source/math/math.cpp"
    "source/geometry/algorithm/2d.cpp"
    "source/geometry/shape.cpp"
    "source/math/linear/vector2.cpp"
    "source/math/linear/vector3.cpp"
    "source/math/linear/matrix2x2.cpp"
    "source/math/linear/matrix3x3.cpp"
    "source/utils/profiler.cpp"
    "source/render/renderer.cpp"
    "include/collision/broadphase/dbvh.h"
    "include/collision/broadphase/aabb.h"
    "source/collision/broadphase/dbvh.cpp"
    "source/collision/broadphase/aabb.cpp"
    "source/render/impl/renderer_qt.cpp"
    
    
    "include/dynamics/joint/joint.h"
    
    "include/dynamics/joint/revolute.h"
    "include/dynamics/joint/pulley.h"
    "include/dynamics/joint/point.h"
    "include/dynamics/joint/distance.h"
    "include/dynamics/joint/rotation.h"
    "include/dynamics/joint/joints.h"    
    "include/dynamics/constraint/contact.h"  
    "include/utils/camera.h" 
    "source/utils/camera.cpp"   
    "include/collision/continuous/ccd.h" 
    "source/collision/continuous/ccd.cpp" 
    "source/utils/random.cpp" 
    "source/dynamics/constraint/contact.cpp" 
    "source/render/impl/renderer_qt.cpp" 
    "include/collision/broadphase/grid.h" 
    "source/collision/broadphase/grid.cpp" 
    "include/collision/broadphase/tree.h" 
    "source/collision/broadphase/tree.cpp" 
    "testbed/frame.h" 
    "testbed/frame.cpp" 
    "testbed/scenes/bitmask.h" 
    "testbed/scenes/geometry.h" 
    "testbed/scenes/collision.h" 
    "testbed/scenes/broadphase.h" 
    "testbed/scenes/narrowphase.h" 
    "testbed/scenes/bridge.h" 
    "testbed/scenes/restitution.h" 
    "testbed/scenes/raycast.h" 
    "testbed/scenes/sensor.h" 
    "testbed/scenes/chain.h" 
    "testbed/scenes/wreckingball.h" "testbed/scenes/newtoncradle.h" "testbed/scenes/pendulum.h")
    
find_package(Qt5 COMPONENTS Widgets REQUIRED)
target_link_libraries(physics-engine PRIVATE Qt5::Widgets)
