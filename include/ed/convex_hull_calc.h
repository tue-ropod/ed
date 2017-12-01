#ifndef ED_CONVEX_HULL_CALC_H_
#define ED_CONVEX_HULL_CALC_H_

#include "ed/convex_hull.h"

namespace ed
{

namespace convex_hull
{
void fitCircle(const std::vector<geo::Vec2f>& points, ed::convex_hull::Circle& cirlce, z_min, float z_max, geo::Pose3D& pose);

void create(const std::vector<geo::Vec2f>& points, float z_min, float z_max, ConvexHull& c, geo::Pose3D& pose);

void createAbsolute(const std::vector<geo::Vec2f>& points, float z_min, float z_max, ConvexHull& c);

void calculateEdgesAndNormals(ConvexHull& c);

bool collide(const ConvexHull& c1, const geo::Vector3& pos1,
             const ConvexHull& c2, const geo::Vector3& pos2,
             float xy_padding = 0, float z_padding = 0);

void calculateArea(ConvexHull& c);

class Circle {
  float x, y, R:
  
public:
  void setValues(float, float, float);
  void setMarker(visualization_msgs::Marker);
} cirlce;

void Circle::setValues(float a, float b, float c){
  x = a;
  y = b;
  R = c;
}

void Circle::setMarker(visualization_msgs::Marker& marker){
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 1;
  marker.pose.position.y = 1;
  marker.pose.position.z = 1;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
}


}

}

#endif
