#ifndef ED_CONVEX_HULL_CALC_H_
#define ED_CONVEX_HULL_CALC_H_

#include "ed/convex_hull.h"
#include <visualization_msgs/Marker.h>

namespace ed
{
  
namespace tracking
{
  class Circle {
  float x_, y_, R_;
  
public:  
  void setValues(float a, float b, float c);
  
  void setMarker(visualization_msgs::Marker& marker);

};





void fitCircle(const std::vector<geo::Vec2f>& points, ed::tracking::Circle* cirlce, float z_min, float z_max, geo::Pose3D& pose); 
}


namespace convex_hull
{

void create(const std::vector<geo::Vec2f>& points, float z_min, float z_max, ConvexHull& c, geo::Pose3D& pose);

void createAbsolute(const std::vector<geo::Vec2f>& points, float z_min, float z_max, ConvexHull& c);

void calculateEdgesAndNormals(ConvexHull& c);

bool collide(const ConvexHull& c1, const geo::Vector3& pos1,
             const ConvexHull& c2, const geo::Vector3& pos2,
             float xy_padding = 0, float z_padding = 0);

void calculateArea(ConvexHull& c);

}

}

#endif
