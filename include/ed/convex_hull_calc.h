#ifndef ED_CONVEX_HULL_CALC_H_
#define ED_CONVEX_HULL_CALC_H_

#include "ed/convex_hull.h"
#include <visualization_msgs/Marker.h>
#include <math.h> 

namespace ed
{
  
namespace tracking
{
  
#define TIMEOUT_TIME 1.0 // [s] TODO: make configurable variable as in the entity cleared (or is it solved automatically by the MHT?!)
#define MAX_LINE_ERROR 0.1 // [m]  
  
  class Circle {
  float x_, y_, R_;
  
public:  
  void setValues(float x, float y, float R);
  
  void setMarker(visualization_msgs::Marker& marker, unsigned int ID);

};

void fitCircle(const std::vector<geo::Vec2f>& points, ed::tracking::Circle* cirlce, geo::Pose3D& pose); 

class Rectangle {
  float x_, y_, w_, h_, theta_; // x, y of center, width and height of rectangle
  
public:  
  void setValues(float x, float y, float w, float h, float theta);
  
  void setMarker(visualization_msgs::Marker& marker, unsigned int ID );

};

void fitRectangle(const std::vector<geo::Vec2f>& points, ed::tracking::Rectangle* rectangle, geo::Pose3D& pose); 

float fitLine(const std::vector<geo::Vec2f>& points, float& angle, geo::Vec2f& centroid, unsigned int& index);

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
