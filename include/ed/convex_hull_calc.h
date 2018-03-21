#ifndef ED_CONVEX_HULL_CALC_H_
#define ED_CONVEX_HULL_CALC_H_

#include "ed/convex_hull.h"
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <numeric>
#include "tf/transform_datatypes.h"

#include <vector>
#include <algorithm>

#include <eigen3/Eigen/Dense>

#include "problib/conversions.h"
#include "problib/datatypes.h"

namespace ed
{

namespace tracking
{

// TODO: make many of variables below configurable/tunable in ED model descriptions?
#define TIMEOUT_TIME 1.0 // [s]
#define MAX_LINE_ERROR 0.05 // [m]  
#define MIN_DISTANCE_CORNER_DETECTION 0.05 // [m]
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
    float x_, y_, z_, R_;
    
    float P_; // estimated covariance of the kalman filter to describe the radius

public:
    void setValues ( float x, float y, float R, float z );
    
    float get_x() { return x_; } ;
    float get_y() { return y_; } ;
    float get_z() { return z_; } ;
    float get_R() { return R_; } ;
    float get_P() { return P_; } ;
    
    void set_x( float x ) { x_ = x; } ;
    void set_y( float y ) { y_ = y; } ;
    void set_z( float z ) { z_ = z; } ;
    void set_R( float R ) { R_ = R; } ;
    void set_P( float P ) { P_ = P; } ;

    void setMarker ( visualization_msgs::Marker& marker, unsigned int ID );

    void printValues();
};

struct greater
{
    template<class T>
    bool operator()(T const &a, T const &b) const { return a < b; }
};

struct laserSegments
{
  std::vector<geo::Vec2f>::iterator begin;
  std::vector<geo::Vec2f>::iterator end;
};

float fitCircle ( std::vector<geo::Vec2f>& points, ed::tracking::Circle* cirlce, geo::Pose3D& pose );

class Rectangle
{
    float x_, y_, z_, w_, d_, h_, theta_; // x, y of center, width, height and rotation of rectangle
    
    Eigen::MatrixXd P_;

public:
    void setValues ( float x, float y, float z, float w, float d, float h, float theta );
    
    float get_x() { return x_; } ;
    float get_y() { return y_; } ;
    float get_z() { return z_; } ;
    float get_w() { return w_; } ;
    float get_d() { return d_; } ;
    float get_theta() { return theta_; } ;
    Eigen::MatrixXd get_P() { return P_; } ;
    
    void set_x( float x ) { x_ = x; } ;
    void set_y( float y ) { y_ = y; } ;
    void set_z( float z ) { z_ = z; } ;
    void set_w( float w ) { w_ = w; } ;
    void set_d( float d ) { d_ = d; } ;
    void set_theta( float theta ) { theta_ = theta; } ;
    void set_P( Eigen::MatrixXd P ) { P_ = P; } ;

    void setMarker ( visualization_msgs::Marker& marker, unsigned int ID );

    void printValues();

};

float fitRectangle ( std::vector<geo::Vec2f>& points, ed::tracking::Rectangle* rectangle, geo::Pose3D& pose , unsigned int cornerIndex );

bool findPossibleCorner ( std::vector< geo::Vec2f >& points, std::vector< unsigned int >* IDs, std::vector< geo::Vec2f >::iterator* it_start, std::vector< geo::Vec2f >::iterator* it_end );

bool findPossibleCorners ( std::vector<geo::Vec2f>& points, std::vector<unsigned int> *cornerIndices );

bool checkForSplit ( std::vector<geo::Vec2f>& points, unsigned int &ID,const geo::Pose3D& sensor_pose,  unsigned int cornerIndex );

float fitLine ( std::vector<geo::Vec2f>& points, Eigen::VectorXd& beta_hat, std::vector<geo::Vec2f>::iterator* it_start, std::vector<geo::Vec2f>::iterator* it_end ) ;//, unsigned int& index);

float setRectangularParametersForLine ( std::vector<geo::Vec2f>& points,  std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, ed::tracking::Rectangle* rectangle, const geo::Pose3D& sensor_pose );

void wrapToInterval ( float* alpha, float lowerBound, float upperBound );

FITTINGMETHOD determineCase ( std::vector<geo::Vec2f>& points, unsigned int* cornerIndex, std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, const geo::Pose3D& sensor_pose );

float fitObject ( std::vector<geo::Vec2f>& points, geo::Pose3D& pose, int FITTINGMETHOD, unsigned int* cornerIndex, ed::tracking::Rectangle* rectangle, ed::tracking::Circle* circle, std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, const geo::Pose3D& sensor_pose);

geo::Vec2f avg ( std::vector<geo::Vec2f>& points, std::vector<geo::Vec2f>::const_iterator it_start, std::vector<geo::Vec2f>::const_iterator it_end );

// Probabilities
class FeatureProbabilities
{
    pbl::PMF pmf_;

public:
    FeatureProbabilities ( float pRectangle_in = 0.5, float pCircle_in = 0.5 ) { // Initialize with 50/50 probabilities
        pmf_.setDomainSize ( 2 );
        pmf_.setProbability ( "Rectangle", pRectangle_in );
        pmf_.setProbability ( "Circle", pCircle_in );
    };

    void setProbabilities ( float pRectangle_in, float pCircle_in ) {
        pmf_.setProbability ( "Rectangle", pRectangle_in );
        pmf_.setProbability ( "Circle", pCircle_in );
    };

    float get_pRectangle() const {
        return ( float ) pmf_.getProbability ( "Rectangle" );
    } ;

    float get_pCircle() const {
        return ( float ) pmf_.getProbability ( "Circle" );
    } ;
    void setMeasurementProbabilities ( float errorRectangleSquared, float errorCircleSquared, float circleRadius, float typicalCorridorWidth );

    void update ( float pRectangle_measured, float pCircle_measured );

    void update ( FeatureProbabilities& featureProbabilities_in );

};

class FeatureProperties
{
    FeatureProbabilities featureProbabilities_; // Probabilities of the features. Is there a need to reset these when there is a switch? Or only when the probability of a feature was low?

    Circle circle_;

    Rectangle rectangle_;

public:
    FeatureProperties ( float pRectangle_in = 0.5, float pCircle_in = 0.5 ) { // Initialize with 50/50 probabilities unless otherwise indicated
        featureProbabilities_.setProbabilities ( pRectangle_in, pCircle_in );
    };

    FeatureProperties ( const FeatureProperties* other ) {
        this->featureProbabilities_ = other->featureProbabilities_;
        this->circle_ = other->circle_;
        this->rectangle_ = other->rectangle_;
    };

    FeatureProbabilities getFeatureProbabilities() const {
        return featureProbabilities_;
    };

    void setFeatureProbabilities ( float pRectangle_in, float pCircle_in ) {
        featureProbabilities_.setProbabilities ( pRectangle_in, pCircle_in );
    };

    void setFeatureProbabilities ( FeatureProbabilities featureProbabilities_in ) {
        featureProbabilities_ = featureProbabilities_in;
    };

    void updateProbabilities ( FeatureProbabilities featureProbabilities_in ) {
        this->featureProbabilities_.update ( featureProbabilities_in );
    };

    void setCircle ( Circle circle_in ) {
        circle_ = circle_in;
    };

    Circle getCircle() const {
        return circle_;
    };

    Rectangle getRectangle() const {
        return rectangle_;
    };

    void setRectangle ( Rectangle rectangle_in ) {
        rectangle_ = rectangle_in;
    };

    void updateCircle(float Q_k, float R_k, float z_k); // z = observation // TODO improve! how? -> x_, y_, R_ Kalman-Filter with constant size model

    void updateRectangle(Eigen::MatrixXd Q_k, Eigen::MatrixXd R_k, Eigen::VectorXd z_k); // TODO improve! how? -> x_, y_, w_, d_, h_, theta_   Kalman-Filter with constant size model

};



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
