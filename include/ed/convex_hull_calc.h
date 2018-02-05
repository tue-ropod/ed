#ifndef ED_CONVEX_HULL_CALC_H_
#define ED_CONVEX_HULL_CALC_H_

#include "ed/convex_hull.h"
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <numeric>
#include "tf/transform_datatypes.h"

#include <vector>
#include <algorithm>


//#include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
namespace ed
{

namespace tracking
{

 // TODO: make many of variables below configurable/tunable in ED model descriptions.  
#define TIMEOUT_TIME 1.0 // [s]
#define MAX_LINE_ERROR 0.05 // [m]  
#define MIN_DISTANCE_CORNER_DETECTION 0.01 // [m]
#define MIN_POINTS_LINEFIT 5 // [-]
#define ARBITRARY_HEIGHT 0.03 //[m]
#define ARBITRARY_DEPTH ARBITRARY_HEIGHT

enum FITTINGMETHOD {
    NONE = 1,
    LINE =   2,
    CIRCLE =   4,
    RECTANGLE =   8
};

class Point
{

public:
    float x, y;

    Point ( double x_in = 0.0, double y_in = 0.0 ) {
        x = x_in;
        y = y_in;
    }

};

class Circle
{
    float x_, y_, R_;

public:
    void setValues ( float x, float y, float R );

    float get_x() {
        return x_;
    } ;
    float get_y() {
        return y_;
    } ;
    float get_R() {
        return R_;
    } ;

    void setMarker ( visualization_msgs::Marker& marker, unsigned int ID,  const geo::Pose3D& sensor_pose );

    void printValues();

};

float fitCircle ( std::vector<geo::Vec2f>& points, ed::tracking::Circle* cirlce, geo::Pose3D& pose );

class Rectangle
{
    float x_, y_, w_, d_, h_, theta_; // x, y of center, width, height and rotation of rectangle

public:
    void setValues ( float x, float y, float w, float d, float h, float theta );

    void setMarker ( visualization_msgs::Marker& marker, unsigned int ID,  const geo::Pose3D& sensor_pose );

    void printValues();

};

float fitRectangle ( std::vector<geo::Vec2f>& points, ed::tracking::Rectangle* rectangle, geo::Pose3D& pose , unsigned int* cornerIndex );

bool findPossibleCorner ( std::vector<geo::Vec2f>& points, unsigned int &ID );

float fitLine ( std::vector<geo::Vec2f>& points, Eigen::VectorXd& beta_hat, std::vector<geo::Vec2f>::iterator it_start, std::vector<geo::Vec2f>::iterator it_end ) ;//, unsigned int& index);

void wrapToInterval ( float* alpha, float lowerBound, float upperBound );

FITTINGMETHOD determineCase ( std::vector<geo::Vec2f>& points, unsigned int* cornerIndex, std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high );

float fitObject ( std::vector<geo::Vec2f>& points, geo::Pose3D& pose, int FITTINGMETHOD,  unsigned int* cornerIndex, ed::tracking::Rectangle* rectangle, ed::tracking::Circle* circle, unsigned int* ID, visualization_msgs::Marker* marker,  std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high );


// Probabilities
// class Rectangle
// {
// }
class probabilitySet
{

public:
    float pRectangle, pCircle;

    probabilitySet ( double pRectangle_in = 0.0, double pCircle_in = 0.0 ) {
        pRectangle = pRectangle_in;
        pCircle = pCircle_in;
    }

};

probabilitySet determineFeatureProbabilities(float errorRectangleSquared, float errorCircleSquared, float circleRadius, float typicalCorridorWidth);


}


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
