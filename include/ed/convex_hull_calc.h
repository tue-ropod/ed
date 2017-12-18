#ifndef ED_CONVEX_HULL_CALC_H_
#define ED_CONVEX_HULL_CALC_H_

#include "ed/convex_hull.h"
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <numeric>
#include "tf/transform_datatypes.h"

#include <vector>
#include <algorithm>


#include <Eigen/Dense>

namespace ed
{
  
namespace tracking
{
  
#define TIMEOUT_TIME 1.0 // [s] TODO: make configurable variable as in the entity cleared (or is it solved automatically by the MHT?!)
#define MAX_LINE_ERROR 0.05 // [m]  
#define MIN_DISTANCE_CORNER_DETECTION 0.1 // [m]
#define MIN_POINTS_LINEFIT 5 // [-]
#define ARBITRARY_HEIGHT 0.01 //[m]
#define ARBITRARY_DEPTH ARBITRARY_HEIGHT

class Point
{

public:
    float x, y;

    Point ( double x_in = 0.0, double y_in = 0.0 ) {
        x = x_in;
        y = y_in;
    }

};

std::vector<float> inscribedRadius ( const std::vector<geo::Vec2f>& points, float* mean, float* std_dev, unsigned int* arg_min );

class Circle {
  float x_, y_, R_;
  
public:  
  void setValues(float x, float y, float R);
  
  void setMarker(visualization_msgs::Marker& marker, unsigned int ID);

};

void fitCircle(const std::vector<geo::Vec2f>& points, ed::tracking::Circle* cirlce, geo::Pose3D& pose); 

class Rectangle {
  float x_, y_, w_, d_, h_, theta_; // x, y of center, width, height and rotation of rectangle
  
public:  
  void setValues(float x, float y, float w, float d, float h, float theta);
  
  void setMarker(visualization_msgs::Marker& marker, unsigned int ID );
  
  void printValues();

};

bool fitRectangle(std::vector<geo::Vec2f>& points, ed::tracking::Rectangle* rectangle, geo::Pose3D& pose); 

bool findCorner(const std::vector<geo::Vec2f>& points, unsigned int &ID);

std::vector<float> fitLine(const std::vector<geo::Vec2f>& points, float& angle, geo::Vec2f& centroid);//, unsigned int& index);

void wrapToInterval(float* alpha, float lowerBound, float upperBound);

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
