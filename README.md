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
  - SAT
  - GJK
  - EPA
  - MPR
  - Distance
  - Contact Pair
  - Support Mapping
    - ellipse
    - circle
    - polygon
    - line
    - point
- Rigid Body Dynamics Simulation
- Axis-Aligned Bounding Box
- Dynamic Bounding Volume Hierarchy
- Basic Debug Drawing
  - Rigid Body
  - AABB
  - DBVH
  - Joint
- Basic 2D Camera
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
  - convexity of polygon
  - the nearest points on line segment and ellipse
  - point on ellipse which is the nearest point to the target point

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
