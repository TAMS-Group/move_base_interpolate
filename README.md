This package contains a simple move_base implementation. Because it does not do any motion planning, it needs a specified obstical free area, in which the robot can safely interpolate between its current position and the target.
This package requires an amcl pose estimation and a map.

List of runtime dependencies:
- [AMCL](http://wiki.ros.org/amcl) for localizing the pr2 on the map
- [map_server](http://wiki.ros.org/map_server) for publishing the map

