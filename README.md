# Physics2D
Simple 2D physics engine for tutoring
# Build
cmake CMakeLists.txt
# Requirement
- C++ 20
- vcpkg
  - Qt
  - fmt

# Features
- Basic Linear Algebra
- Collision Detection Algorithm
  - SAT
  - GJK
  - EPA
  - MPR
  - Distance
  - Contact Pair
  - Support Mapping
    - Ellipse
    - Circle
    - Polygon
    - Line
    - Point
    - Capsule
- Continuous Collision Detection
  - Sampling Trajectory of Body
  - Time of Impact
- Rigid Body Dynamics Simulation
- Axis-Aligned Bounding Box
- Dynamic Bounding Volume Tree
  - Dynamic Tree
  - Dynamic Array
- Basic Debug Drawing
  - Rigid Body
  - AABB
  - DBVH
  - Joint
- Basic 2D Camera
  - Zooming
  - Smooth Transition
  - Tracing Specified Body
- Simple 2D Geometry Algorithm
  - Intersection
    - Raycast
    - Line Segment
  - Convexity
    - Graham Scan
    - Convexity Test
  - Center
    - Incenter
    - Centroid
    - Circumcenter
  - Circle
    - Circumcircle
    - Inscribed-circle
  - Ellipse
    - Support Mapping
    - Nearest Point

# Future
- Sequential Impulse Solver
- Joint
  - Mouse
  - Prismatic
  - Angle
  - Point
  - Weld
  - Distance
- Non-Fit Polygon
- Finite Element Method
- Mass-Spring System
- Test Demo
