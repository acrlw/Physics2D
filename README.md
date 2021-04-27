# Physics2D
Physics2D - Simple 2D Physics Engine for tutoring
# Build
cmake CMakeLists.txt
# Requirement
- C++ 17
- vcpkg
  - Qt
  - fmt

# Features
- Basic Linear Algebra
- Collision Detection Algorithm
  - GJK
  - EPA
  - Distance
  - Contact Pair
  - Support Shape
    - ellipse
    - circle
    - polygon
    - line
    - point
- Simple 2D Geometry Algorithm
  - raycast
  - graham scan convex hull
  - mass menter of polygon
  - triangle incenter
  - triangle centroid
  - triangle circumcenter
  - line segment intersection
  - circumcircle of triangle
  - inscribed circle of triangle
  - check convexity of polygon
  - calculate the nearest points on line segment and ellipse
  - calculate point on ellipse which is the nearest point to the target point
- Rigid Body Dynamics Simulation
- Axis-Aligned Bounding Box Generator

# Future
- Sequential Impulse Solver
- Joint
  - Mouse
  - Prismatic
  - Angle
  - Point
  - Weld
  - Distance
- Dynamic Bounding Volume Hierarchy Generator
