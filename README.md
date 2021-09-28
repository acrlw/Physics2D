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
    - Algorithm
      - SAT
      - GJK & EPA & MPR & Distance & Contact Pair
    - Continuous Collision Detection
      - Sampling Trajectory of Body
      - Time of Impact
  - Broadphase
    - Axis-Aligned Bounding Box
    - Dynamic Bounding Volume Tree
      - SAH
      - Tree & Array
- Contact Maintainer
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
  - Support Mapping
      - Ellipse
      - Circle
      - Polygon
      - Line
      - Point
      - Capsule
      - Sector
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

# Reference
- [Box2D](https://github.com/erincatto/box2d)
- [Box2D Lite](https://github.com/erincatto/box2d-lite)
- [Box2D Publications](https://box2d.org/publications/)
- [dyn4j Official Blog](https://dyn4j.org/blog/)
- [dyn4j](https://github.com/dyn4j/dyn4j)
- [Allen Chou's Blog](http://allenchou.net/game-physics-series/)
- [Gaffer's on Games](https://gafferongames.com/#posts)
- [*Introduction to rigid body pipeline, collision detection* - Erwin Coumans](https://docs.google.com/presentation/d/1wGUJ4neOhw5i4pQRfSGtZPE3CIm7MfmqfTp5aJKuFYM/edit#slide=id.g644a5aa5f_1_116)
- *Foundations of Physically Based Modeling and Animation* - Donald House and John C. Keyser
