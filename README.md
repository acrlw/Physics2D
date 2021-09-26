# Physics2D
Simple 2D Physics Engine For Tutoring.
# Build
cmake CMakeLists.txt
# Requirement
- C++ 20
- vcpkg
  - Qt
  - fmt

# Features
- Basic Linear Algebra
- Collision Detection
  - Narrowphase
    - Basic Algorithm
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
  - Broadphase
    - Axis-Aligned Bounding Box
    - Dynamic Bounding Volume Tree
      - SAH
      - Tree & Array
- Contact Cache
- Rigid Body Dynamics Simulation
- Sequential Impulse Solver
- Joint
  - Distance
  - Rotation
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
- Broadphase
  - Uniform Grid
- Test Demo
- Integrator
  - Verlet
  - Rk4
- Joint
  - Mouse
  - Prismatic
  - Point
  - Weld
- Non-Fit Polygon
- Soft Body
  - Finite Element Method
  - Mass-Spring System
