#ifndef ED_CONVEX_HULL_CALC_H_
#define ED_CONVEX_HULL_CALC_H_

#include "ed/convex_hull.h"
//#include "ros/ros.h" // used for the msg timestamps, need to be removed if markers are not communicated via ros anymore TODO
//#include <visualization_msgs/Marker.h>
//#include <math.h>
//#include <numeric>
//#include "tf/transform_datatypes.h"

//#include <vector>
//#include <algorithm>

// #include <eigen3/Eigen/Dense>
//#include <Eigen/Dense>

//#include "problib/conversions.h"
//#include "problib/datatypes.h"
//#include "ed/termcolor.hpp"

//#include <sensor_msgs/LaserScan.h>
//#include <geolib/sensors/LaserRangeFinder.h>

namespace ed
{



namespace convex_hull
{

void create ( const std::vector<geo::Vec2f>& points, float z_min, float z_max, ConvexHull& c, geo::Pose3D& pose );

void createAbsolute ( const std::vector<geo::Vec2f>& points, float z_min, float z_max, ConvexHull& c );

void calculateEdgesAndNormals ( ConvexHull& c );

bool collide ( const ConvexHull& c1, const geo::Vector3& pos1,
               const ConvexHull& c2, const geo::Vector3& pos2,
               float xy_padding = 0, float z_padding = 0 );

void calculateArea ( ConvexHull& c );

}

}

#endif
