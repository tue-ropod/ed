#include "ed/convex_hull_calc.h"

#include <opencv2/imgproc/imgproc.hpp>

namespace ed
{

namespace tracking
{
void wrap2Interval ( float* alpha, float lowerBound, float upperBound )
{
    float delta = upperBound - lowerBound;

    if ( *alpha < lowerBound )
    {
        while ( *alpha < lowerBound )
        {
            *alpha += delta;
        }
    }
    else if ( *alpha >= upperBound )
    {
        while ( *alpha >= upperBound )
        {
            *alpha -= delta;
        }
    }
}

template <typename T> int sgn(T val) 
{
    return (T(0) < val) - (val < T(0));
}

void unwrap (float *angleMeasured, float angleReference, float increment)
{
        // Rectangle is symmetric over pi-radians, so unwrap to pi
        float diff = angleReference - *angleMeasured;
        
        int d = diff / (increment);
        *angleMeasured += d*increment;
        
        float r = angleReference - *angleMeasured;
        
        if( fabs(r) > (0.5*increment) )
        {
                *angleMeasured += sgn(r)*increment;
        }
}

FITTINGMETHOD determineCase ( std::vector<geo::Vec2f>& points, unsigned int* cornerIndex, std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, const geo::Pose3D& sensor_pose )
{
    // Determine is a line or a rectangle should be fitted. In case of a rectangle, the number of elements for both sides should meet the minimum number of points for a line fit
    // for both lines. Otherwise, a line will be fitted on the remaining points.

    *it_low = points.begin();
    *it_high = points.end();

    bool includeCorner = *cornerIndex > 0;

    // In the case of including a corner, check if both segments have enough points to describe a line with. If not, do not use these data.
    if ( includeCorner ) {
        unsigned int nPointsLow = *cornerIndex + 1; // + 1 because the corner can be used for both lines
        unsigned int nPointsHigh = points.size() - *cornerIndex;
        unsigned int remainingSize = points.size();

        bool fitSingleline = false;
        bool pointsRemoved = false;

        if ( nPointsLow < MIN_POINTS_LINEFIT ) 
        { // Part of section too smal -> remove it from the data which are analyzed and try to fit line again
            *it_low += *cornerIndex;
            remainingSize = nPointsHigh;
            pointsRemoved = true;
        }
        
        if ( nPointsHigh < MIN_POINTS_LINEFIT ) 
        {
            *it_high -= nPointsHigh;
            remainingSize = nPointsLow;
            pointsRemoved = true;
        }       

        if ( pointsRemoved && remainingSize < MIN_POINTS_LINEFIT ) 
        {
            *cornerIndex = std::numeric_limits<unsigned int>::quiet_NaN();
	    return NONE;
	    
        }
        else if ( pointsRemoved && remainingSize >= MIN_POINTS_LINEFIT ) 
        {
            *cornerIndex = std::numeric_limits<unsigned int>::quiet_NaN();
            return LINE;
        }
        else  
        { // we dit not remove points and a corner is present
            return RECTANGLE;
        }
    }
    else 
    {
        return LINE;
    }
    return NONE;
}

float fitObject ( std::vector<geo::Vec2f>& points, int FITTINGMETHOD,  unsigned int* cornerIndex, ed::tracking::Rectangle* rectangle, ed::tracking::Circle* circle, std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, const geo::Pose3D& sensor_pose )
{
    switch ( FITTINGMETHOD )
    {
    case NONE:
    {
        return std::numeric_limits<float>::infinity();
    }
    case LINE:
    {
        return setRectangularParametersForLine ( points,  it_low,  it_high, rectangle, sensor_pose );
    }
    case CIRCLE:
    {
        return fitCircle ( points, circle, sensor_pose );
    }
    case RECTANGLE:
    {
        return fitRectangle ( points, rectangle, sensor_pose , *cornerIndex );
    }
    }
    return false; // end reached without doing something
}

bool determineCornerConfidence(const sensor_msgs::LaserScan::ConstPtr& scan, unsigned int element, bool elementLow) 
// if !elementLow, element = elementHigh;
{
        unsigned int num_beams = scan->ranges.size();
        unsigned int nPointsToCheck = POINTS_TO_CHECK_CONFIDENCE;
        
        if(elementLow)
        {
                if ( element < nPointsToCheck )
                {
                        // Because we have no proof that the complete side of the object is observed
                        return false;
                }
                else
                {
                        
                        float rsToCheck = scan->ranges[element];
                        for ( unsigned int l = element - nPointsToCheck; l < element; l++ )
                        {
                                float rsToCompare = scan->ranges[l];
                                if ( rsToCheck > rsToCompare + LASER_ACCURACY && rsToCompare >= 0 + EPSILON )
                                {
                                        return false;
                                }
                        }
                }
                return true;    
        }
        
        else // we need to analyse elementHigh
        {
        
                if ( num_beams - element < nPointsToCheck )
                {
                        return false;
                }
                else
                {
                        float rsToCheck = scan->ranges[element];
                        
                        for ( unsigned int l = element; l < element + nPointsToCheck; l++ )
                        {
                                float rsToCompare = scan->ranges[l];
                                if ( rsToCheck > rsToCompare + LASER_ACCURACY && rsToCompare >= 0 + EPSILON )
                                {
                                        return false;
                                }
                        }     
                }
                
                return true; 
        }
}

geo::Vec2f avg ( std::vector<geo::Vec2f>& points, std::vector<geo::Vec2f>::const_iterator it_start, std::vector<geo::Vec2f>::const_iterator it_end )
{
    geo::Vec2f avg_point;
    avg_point.x = avg_point.y= 0.0;

    for ( std::vector<geo::Vec2f>::const_iterator it = it_start; it != it_end; ++it ) {
        geo::Vec2f point = *it;
        avg_point.x += point.x;
        avg_point.y += point.y;
    }

    unsigned int nElements = std::distance ( it_start, it_end );
    avg_point.x /= nElements;
    avg_point.y /= nElements;

    return ( avg_point );
}

geo::Vec2f projectPointOnLine(geo::Vec2f p1Line, geo::Vec2f p2Line, geo::Vec2f point2Project)
{
        float x1 = p1Line.x, x2 = p2Line.x, x3 = point2Project.x;
        float y1 = p1Line.y, y2 = p2Line.y, y3 = point2Project.y;
        
        float factor = ((y2-y1) * (x3-x1) - (x2-x1) * (y3-y1)) / (pow(y2-y1, 2.0) + pow(x2-x1, 2.0)); // https://stackoverflow.com/questions/1811549/perpendicular-on-a-line-from-a-given-point
        
        geo::Vec2f intersection;
        intersection.x = x3 - factor * (y2-y1); // Now, x, y corrected is on the edge in the depth dimension. The position still need to be corrected in the width dimension.
        intersection.y = y3 + factor * (x2-x1);
        
        return intersection;
}

Eigen::MatrixXf kalmanUpdate(Eigen::MatrixXf F, Eigen::MatrixXf H, Eigen::MatrixXf *P, Eigen::MatrixXf x_k_1_k_1, Eigen::MatrixXf z_k, Eigen::MatrixXf Q, Eigen::MatrixXf R)
{
    Eigen::MatrixXf I;
    I.setIdentity ( F.rows(), F.cols() );
    Eigen::MatrixXf x_k_k_1 = F*x_k_1_k_1;
    Eigen::MatrixXf P_k_k_1 = F* (*P) * F.transpose() + Q;
    Eigen::MatrixXf y_k = z_k - H*x_k_k_1;
    Eigen::MatrixXf S_k = H*P_k_k_1*H.transpose() + R;
    Eigen::MatrixXf K_k = P_k_k_1*H.transpose() *S_k.inverse();
    Eigen::MatrixXf x_k_k = x_k_k_1 + K_k*y_k;
    Eigen::MatrixXf P_k_k = ( I - K_k*H ) *P_k_k_1;  
    
    *P = P_k_k;
    
    return x_k_k;
}

//Fast Line, Arc/Circle and Leg Detection from Laser Scan Data in a Player Driver: http://miarn.sourceforge.net/pdf/a1738b.pdf
float fitCircle ( std::vector<geo::Vec2f>& points, ed::tracking::Circle* circle, const geo::Pose3D& pose )
{
    // according to https://dtcenter.org/met/users/docs/write_ups/circle_fit.pdf
    float x_avg = 0.0, y_avg = 0.0;
    for ( unsigned int i = 0; i < points.size(); ++i ) {
        x_avg += points[i].x;
        y_avg += points[i].y;
    }

    x_avg /= points.size();
    y_avg /= points.size();

    std::vector<float> ui ( points.size() ), vi ( points.size() );
    float Suu = 0.0, Suv = 0.0, Suuu = 0.0, Suvv = 0.0, Svv = 0.0, Svvv = 0.0, Svuu = 0.0;
    for ( unsigned int i = 0; i < points.size(); ++i ) {
        ui[i] = points[i].x -x_avg;
        vi[i] = points[i].y -y_avg;

        Suu += ui[i]*ui[i];
        Suv += ui[i]*vi[i];
        Suuu += ui[i]*ui[i]*ui[i];
        Suvv += ui[i]*vi[i]*vi[i];

        Svv += vi[i]*vi[i];
        Svvv += vi[i]*vi[i]*vi[i];
        Svuu += vi[i]*ui[i]*ui[i];
    }

    float a = Suu;
    float b = Suv;
    float c = 0.5* ( Suuu+Suvv );
    float d = Suv;
    float e = Svv;
    float f = 0.5* ( Svvv+Svuu );

    float vc = ( f - c*d/a ) / ( e-b*d/a );
    float uc = ( c-vc*b ) /a;

    float xc = uc+x_avg;
    float yc = vc+y_avg;

    float alpha = uc*uc+vc*vc+ ( Suu+Svv ) /points.size();
    float radius = std::sqrt ( alpha );
     
    float sum = 0.0;
    for ( unsigned int i = 0; i < points.size(); ++i ) 
    {
        float error = fabs ( sqrt ( pow ( xc - points[i].x, 2.0 ) + pow ( yc - points[i].y, 2.0 ) ) - radius ); // distance between a point and a circle;
        float error2 = pow ( error, 2.0 );
        sum += error2;
    }

    float roll = 0.0, pitch = 0.0, yaw = 0.0;
    circle->setProperties ( xc, yc, pose.getOrigin().getZ(), roll, pitch, yaw, radius); // Assumption: object-height identical to sensor-height
    return sum/points.size();
}

Circle::Circle()
{
    float notANumber = 0.0/0.0;
    P_.setIdentity( 4, 4 ); 
    Pdim_.setIdentity( 1, 1 ); 
    this->setProperties( notANumber, notANumber, notANumber, notANumber, notANumber, notANumber, notANumber  ); // Produces NaN values, meaning that the properties are not initialized yet
    xVel_   = 0.0;
    yVel_   = 0.0;
}

void Circle::setProperties ( float x, float y, float z, float roll, float pitch, float yaw, float radius )
{
    x_ = x;
    y_ = y;
    z_ = z;
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
    radius_ = radius;
}

void Circle::printProperties ( )
{
    std::cout << "x_ = " << x_;
    std::cout << " y_ = " << y_;
    std::cout << " xVel_ = " << xVel_;
    std::cout << " yVel_ = " << yVel_;
    std::cout << " roll_ = " << roll_;
    std::cout << " pitch_ = " << pitch_;
    std::cout << " yaw_ = " << yaw_;
    std::cout << " radius = " << radius_ ;
    std::cout << " P_ = " << P_;
    std::cout << "Pdim_ = " << Pdim_ << std::endl;
}

void Circle::setMarker ( visualization_msgs::Marker& marker , unsigned int ID )
{
   std_msgs::ColorRGBA color;
   color.a = 0.5;
   color.r = 0.0;
   color.g = 1.0;
   color.b = 0.0;
   
   this->setMarker ( marker, ID, color ); 
}

void Circle::setMarker ( visualization_msgs::Marker& marker, unsigned int ID, std_msgs::ColorRGBA color )
{
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Position Marker";
    marker.id = ID;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = z_;
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( roll_, pitch_, yaw_ );
    marker.scale.x = 2*radius_;
    marker.scale.y = 2*radius_;
    marker.scale.z = 0.1;
    
    marker.color = color;

    marker.lifetime = ros::Duration ( TIMEOUT_TIME );
}

void Circle::setTranslationalVelocityMarker( visualization_msgs::Marker& marker, unsigned int ID )
{
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Translational Velocity Marker";
    marker.id = ID;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = z_;

    // Pivot point is around the tip of its tail. Identity orientation points it along the +X axis. 
    // scale.x is the arrow length, scale.y is the arrow width and scale.z is the arrow height.     
    float rollVel = 0.0;
    float pitchVel = 0.0;
    float yawVel = atan2( yVel_, xVel_ );
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollVel, pitchVel, yawVel );

    marker.scale.x = sqrt( pow(xVel_, 2.0) + pow(yVel_, 2.0) );
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
}

std::vector< geo::Vec2f > Circle::convexHullPoints(unsigned int nPoints)
{
  std::vector< geo::Vec2f > Points(nPoints);
  float deltaAngle = 2*M_PIl / nPoints;
  
  for(unsigned int ii = 0; ii < nPoints; ii++)
  {
    float angle = ii*deltaAngle;
    Points[ii].x = x_ + radius_*cos(angle);
    Points[ii].y = y_ + radius_*sin(angle);
  }
  
  return Points;
}

float Circle::predictX( float dt )
{
        return x_ + dt * xVel_;
}

float Circle::predictY( float dt )
{
        return y_ + dt * yVel_;
}

void Circle::predictPos( float* predictedX, float* predictedY, float dt )
{
        *predictedX = predictX( dt );
        *predictedY = predictY( dt );
}

void Circle::predictAndUpdatePos( float dt )
{
        x_ = predictX( dt );
        y_ = predictY( dt );
}


float fitRectangle ( std::vector<geo::Vec2f>& points, ed::tracking::Rectangle* rectangle, const geo::Pose3D& pose , unsigned int cornerIndex )
{
    std::vector<geo::Vec2f>::iterator it_start = points.begin();
    std::vector<geo::Vec2f>::iterator it_splitLow = points.begin() + cornerIndex - 1; // +1 and -1 to prevent problems when a corner is not visible due to an occlusion
    std::vector<geo::Vec2f>::iterator it_splitHigh = points.begin() + cornerIndex + 1;
    std::vector<geo::Vec2f>::iterator it_end = points.end();

    Eigen::VectorXf beta_hat1 ( 2 ), beta_hat2 ( 2 );
    bool validFit1, validFit2;

    float mean_error1 = fitLine ( points, beta_hat1, &it_start, &it_splitLow ); //
    float mean_error2 = fitLine ( points, beta_hat2, &it_splitHigh, &it_end );

    float x_start1 = points[0].x; // Is this correct in combination with theta?
    float y_start1 = points[0].y;

    // determine width and height
    float x_end = points[cornerIndex - 1].x;
    float y_end = points[cornerIndex - 1].y;
    
    float theta = atan2 ( beta_hat1 ( 1 ), 1 ); // TODO: angle on points low alone?

    float x_start2 = points[cornerIndex + 1].x;
    float y_start2 = points[cornerIndex + 1].y;
    
    float x_end2 = points.back().x;
    float y_end2 = points.back().y;
   
    float dx = x_end2 - x_start2;
    float dy = y_start2 - y_end2;
    float depth = sqrt ( dx*dx+dy*dy ); // minimal depth
    
    // As we might have partial detection in both width and depth direction, ensure that the dimension and positions are corrected according to this
    float centerWidth_x = 0.5* ( x_start1 + x_end );
    float centerWidth_y = 0.5* ( y_start1 + y_end );
    
    float centerDepth_x = 0.5* ( x_end2 + x_start2 );
    float centerDepth_y = 0.5* ( y_end2 + y_start2  );    
    
    geo::Vec2f p1Line, p2Line, point2Project;
    float ct = cos ( theta );
    float st = sin ( theta );
    
    p1Line.x = centerDepth_x - 0.5*depth*st;
    p1Line.y = centerDepth_y + 0.5*depth*ct;
    
    p2Line.x = centerDepth_x -(- 0.5*depth*st);
    p2Line.y = centerDepth_y -(+ 0.5*depth*ct);
    
    point2Project.x = centerWidth_x;
    point2Project.y = centerWidth_y;
    
    geo::Vec2f cornerPoint = projectPointOnLine( p1Line, p2Line, point2Project);

    float width = 2*( sqrt ( pow( centerWidth_x - cornerPoint.x, 2.0) + pow( centerWidth_y - cornerPoint.y, 2.0) ) ); 
    depth = 2*( sqrt ( pow( centerDepth_x - cornerPoint.x, 2.0) + pow( centerDepth_y - cornerPoint.y, 2.0) ) );
    
    float center_x = 0.5* ( x_start1 + x_end ) + 0.5* ( x_end2 - x_start2 ); // uncorrected
    float center_y = 0.5* ( y_start1 + y_end ) + 0.5* ( y_end2 - y_start2 );
    
    float center_x_correctedPos = centerDepth_x + 0.5*width*ct;
    float center_y_correctedPos = centerDepth_y + 0.5*width*st;
    
    float center_x_correctedNeg = centerDepth_x - 0.5*width*ct;
    float center_y_correctedNeg = centerDepth_y - 0.5*width*st;
    
    float dist2Pos = pow( center_x_correctedPos - center_x, 2.0) + pow( center_y_correctedPos - center_y, 2.0);
    float dist2Neg = pow( center_x_correctedNeg - center_x, 2.0) + pow( center_y_correctedNeg - center_y, 2.0);
    
    if( dist2Pos < dist2Neg )
    {
            center_x = center_x_correctedPos;
            center_y = center_y_correctedPos;
    }
    else
    {
            center_x = center_x_correctedNeg;
            center_y = center_y_correctedNeg;
    }
    
    float roll = 0.0, pitch = 0.0, yaw = theta;
    rectangle->setValues ( center_x, center_y, pose.getOrigin().getZ(), width, depth, ARBITRARY_HEIGHT, roll, pitch, yaw ); // Assumption: object-height identical to sensor-height

    unsigned int low_size = cornerIndex;
    unsigned int high_size = points.size() - cornerIndex + 1;
    
    return ( mean_error1*low_size+mean_error2*high_size ) / ( low_size + high_size ); // average of error
}

bool findPossibleCorner ( std::vector<geo::Vec2f>& points, std::vector<unsigned int> *IDs, std::vector<geo::Vec2f>::iterator* it_start, std::vector<geo::Vec2f>::iterator* it_end )
{
    float maxDistance = 0.0;
    unsigned int ID = std::numeric_limits<unsigned int>::quiet_NaN();

    geo::Vec2f startPoint = **it_start;
    geo::Vec2f endPoint = * ( *it_end - 1 );

    float a = endPoint.y-startPoint.y;
    float b = endPoint.x-startPoint.x;
    float c = endPoint.x*startPoint.y-endPoint.y*startPoint.x;

    float length = sqrt ( pow ( a,2.0 ) + pow ( b,2.0 ) );

    // See https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    for ( std::vector<geo::Vec2f>::iterator it = *it_start + 1; it != *it_end - 1; ++it ) 
    {

        geo::Vec2f point = *it;
        float distance = fabs ( a* ( point.x )-b* ( point.y ) +c ) / length;

        if ( distance > maxDistance ) 
        {
            maxDistance = distance;
            ID = std::distance ( points.begin(), it );
        }
    }

    if ( maxDistance >  MIN_DISTANCE_CORNER_DETECTION ) 
    {
        IDs->push_back ( ID );
        return true;
    } 
    else 
    {
        return false;
    }

}

bool findPossibleCorners ( std::vector<geo::Vec2f>& points, std::vector<unsigned int> *cornerIndices )
{
    // Check in section if a corner is present. If that is the case, split the data at this corner, and check for both parts if another corner is present.
    std::vector<laserSegments> segments;

    std::vector<geo::Vec2f>::iterator it_start = points.begin();
    std::vector<geo::Vec2f>::iterator it_end = points.end();

    if ( findPossibleCorner ( points, cornerIndices, &it_start, &it_end ) )
    { // -1 because std::vector::end returns an iterators to one-past-the-end of the container. The element just before is then the last element in the vector.
        laserSegments segmentToAdd;
        segmentToAdd.begin = points.begin();
        segmentToAdd.end = points.begin() + cornerIndices->back();

        if ( segmentToAdd.end - segmentToAdd.begin > MIN_POINTS_LINEFIT ) 
        {
            segments.push_back ( segmentToAdd );
        }

        segmentToAdd.begin = points.begin() + cornerIndices->back();
        segmentToAdd.end = points.end() - 1;

        if ( segmentToAdd.end - segmentToAdd.begin > MIN_POINTS_LINEFIT ) 
        {
            segments.push_back ( segmentToAdd );
        }

        for ( unsigned int ii = 0; ii < segments.size(); ++ii ) 
        {
            laserSegments laserSegment = segments[ii];
            geo::Vec2f pointEnd =  *laserSegment.end;

            bool test = findPossibleCorner ( points, cornerIndices, &laserSegment.begin, &laserSegment.end );

            if ( test ) 
            {
                segmentToAdd.begin = laserSegment.begin;
                segmentToAdd.end = points.begin() + cornerIndices->back();

                if ( segmentToAdd.end - segmentToAdd.begin > MIN_POINTS_LINEFIT ) 
                {
                    segments.push_back ( segmentToAdd );
                }

                segmentToAdd.begin = points.begin() + cornerIndices->back();
                segmentToAdd.end = laserSegment.end;

                if ( segmentToAdd.end - segmentToAdd.begin > MIN_POINTS_LINEFIT ) 
                {
                    segments.push_back ( segmentToAdd );
                }
            }
        }

        std::sort ( cornerIndices->begin(), cornerIndices->end(), greater() );
        
        return true;
    } 
    else 
    {
        return false;
    }
}

bool checkForSplit ( std::vector<geo::Vec2f>& points, const geo::Pose3D& sensor_pose,  unsigned int cornerIndex )
{
    // check if a split is required: 2 objects close to each other can form a rectangle in the wrong quadrant. Model as 2 separate lines
    geo::Vec2f centerpoint;
    centerpoint.x = 0.5* ( points[0].x + points[points.size() - 1].x );
    centerpoint.y = 0.5* ( points[0].y + points[points.size() - 1].y );

    float centerDist2 = pow ( sensor_pose.getOrigin().getX() - centerpoint.x, 2.0 ) + pow ( sensor_pose.getOrigin().getY() - centerpoint.y, 2.0 );
    float cornerDist2 = pow ( sensor_pose.getOrigin().getX() - points[cornerIndex].x, 2.0 ) + pow ( sensor_pose.getOrigin().getY() - points[cornerIndex].y, 2.0 );

    if ( centerDist2 < cornerDist2 ) 
    {
        return true;
    }
    else 
    {
        return false;
    }
}

float fitLine ( std::vector<geo::Vec2f>& points, Eigen::VectorXf& beta_hat, std::vector<geo::Vec2f>::iterator* it_start, std::vector<geo::Vec2f>::iterator* it_end )  //, unsigned int& index )
{
    // Least squares method: http://home.isr.uc.pt/~cpremebida/files_cp/Segmentation%20and%20Geometric%20Primitives%20Extraction%20from%202D%20Laser%20Range%20Data%20for%20Mobile%20Robot%20Applications.pdf
    unsigned int size = std::distance ( *it_start, *it_end );
    Eigen::MatrixXf m ( size, 2 );
    Eigen::VectorXf y ( size );
    unsigned int counter = 0;
    
    std::vector<geo::Vec2f>::iterator it = *it_start;
    geo::Vec2f point_start = *it;
    it = *it_end; it--;
    geo::Vec2f point_end = *it;
    
    for ( std::vector<geo::Vec2f>::iterator it = *it_start; it != *it_end; ++it ) 
    {
        geo::Vec2f point = *it;
        m ( counter, 0 ) = ( double ) 1.0;
        m ( counter, 1 ) = ( double ) point.x;
        y ( counter ) = ( double ) point.y;
        counter++;
    }

    Eigen::MatrixXf mt ( size, 2 );
    mt = m.transpose();

    beta_hat = ( mt*m ).inverse() * mt * y;

    float error, sum = 0.0;

    counter = 0;
    for ( std::vector<geo::Vec2f>::iterator it = *it_start; it != *it_end; ++it ) 
    {
        // Distance of each point to line
        geo::Vec2f point = *it;
        error = fabs ( -beta_hat ( 1 ) * point.x+point.y - beta_hat ( 0 ) ) /sqrt ( beta_hat ( 1 ) *beta_hat ( 1 ) + 1 ); // distance of a point to a line, see https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        float error2 = pow ( error, 2.0 );
        sum += error2;
        counter ++;
    }

    return sum/counter;
}

float setRectangularParametersForLine ( std::vector<geo::Vec2f>& points,  std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high, ed::tracking::Rectangle* rectangle, const geo::Pose3D& sensor_pose )
{
    Eigen::VectorXf beta_hat ( 2 );
    float mean_error2 = fitLine ( points, beta_hat, it_low, it_high ) ;

    float theta = atan2 ( beta_hat ( 1 ), 1 );

    unsigned int ii_start = std::distance ( points.begin(), *it_low );
    
    float x_start = points[ii_start].x;
    float y_start = points[ii_start].y; // better to rely on the original points: extreme outliers are already filtered out due to segmentation

    unsigned int ii_end = std::distance ( points.begin(), *it_high );
    float x_end = points[ii_end - 1].x; // considered to be rebust to rely on 1 point, as these are filtered out already during the segmantation phase
    float y_end = points[ii_end - 1].y;

    float dx = x_end - x_start;
    float dy = y_start - y_end;
    float width = sqrt ( dx*dx+dy*dy );

    float center_x = 0.5* ( x_start + x_end );
    float center_y = 0.5* ( y_start + y_end );

    float roll = 0.0, pitch = 0.0, yaw = theta;
    
    rectangle->setValues ( center_x, center_y, sensor_pose.getOrigin().getZ(), width, ARBITRARY_DEPTH, ARBITRARY_HEIGHT, roll, pitch, yaw ); // Assumption: object-height identical to sensor-height

    return mean_error2;
}

Rectangle::Rectangle()
{
    float notANumber = 0.0/0.0;
    P_.setIdentity( 6, 6 ); 
    Pdim_.setIdentity( 2, 2 ); 
    this->setValues( notANumber, notANumber, notANumber, notANumber, notANumber, notANumber, notANumber, notANumber, notANumber ); // Produces NaN values, meaning that the properties are not initialized yet
    xVel_   = 0.0;
    yVel_   = 0.0;
    yawVel_ = 0.0;
}

void Rectangle::setValues ( float x, float y, float z, float w, float d, float h, float roll, float pitch, float yaw )
{
    x_ = x;
    y_ = y;
    z_ = z;
    w_ = w;
    d_ = d;
    h_ = h;
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
}

void Rectangle::printProperties ( )
{
    std::cout << "x_ = "      << x_;
    std::cout << " y_ = "     << y_;
    std::cout << " z_ = "     << z_;
    std::cout << " w_ = "     << w_;
    std::cout << " d_ = "     << d_;
    std::cout << " h_ = "     << h_;
    std::cout << " xVel_ = "  << xVel_;
    std::cout << " yVel_ = "  << yVel_;
    std::cout << " yawVel_ = "<< yawVel_;
    std::cout << " roll_ = "  << roll_;
    std::cout << " pitch_ = " << pitch_;
    std::cout << " yaw_ = "   << yaw_;
}

float Rectangle::predictX( float dt )
{
        return x_ + dt * xVel_;
}

float Rectangle::predictY( float dt )
{
        return y_ + dt * yVel_;
}

float Rectangle::predictYaw( float dt )
{
        return yaw_ + dt * yawVel_;
}

void Rectangle::predictPos( float* predictedX, float* predictedY, float* predictedYaw, float dt )
{
        *predictedX = predictX( dt );
        *predictedY = predictY( dt );
        *predictedYaw = predictYaw( dt );
}

void Rectangle::predictAndUpdatePos( float dt )
{
        x_ = predictX( dt );
        y_ = predictY( dt );
        yaw_ = predictYaw( dt );
}

void Rectangle::setMarker ( visualization_msgs::Marker& marker, unsigned int ID, std_msgs::ColorRGBA color, std::string ns )
{
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = ID;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = z_;
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( roll_, pitch_, yaw_ );
    marker.scale.x = w_;
    marker.scale.y = d_;
    marker.scale.z = 0.1;
    marker.color = color;
    marker.lifetime = ros::Duration ( TIMEOUT_TIME );
}

void Rectangle::setMarker ( visualization_msgs::Marker& marker , unsigned int ID )
{
   std_msgs::ColorRGBA color;
   color.a = 0.5;
   color.r = 0.0;
   color.g = 1.0;
   color.b = 0.0;
   
   this->setMarker ( marker, ID, color ); 
}

void Rectangle::setMarker ( visualization_msgs::Marker& marker, unsigned int ID, std_msgs::ColorRGBA color)
{
        this->setMarker ( marker, ID, color, "Position Marker" ); 
}

void Rectangle::setTranslationalVelocityMarker( visualization_msgs::Marker& marker, unsigned int ID )
{
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Translational Velocity Marker";
    marker.id = ID;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = z_;
    
    float rollTranslationalVel = 0.0;
    float pitchTranslationalVel = 0.0;
    float yawTranslationalVel = std::atan2( yVel_, xVel_ );
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollTranslationalVel, pitchTranslationalVel, yawTranslationalVel );

   // Pivot point is around the tip of its tail. Identity orientation points it along the +X axis. 
   // scale.x is the arrow length, scale.y is the arrow width and scale.z is the arrow height.     
    marker.scale.x = std::sqrt( std::pow(xVel_, 2.0) + std::pow(yVel_, 2.0) );
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
}

void Rectangle::setRotationalVelocityMarker( visualization_msgs::Marker& marker, unsigned int ID )
{
    // At the first corner, place an indicator about the rotational vel
    std::vector<geo::Vec2f> corners = determineCorners( 0.0 );
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Rotational Velocity Marker";
    marker.id = ID;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = corners[0].x;
    marker.pose.position.y = corners[0].y;
    marker.pose.position.z = z_;

    // Pivot point is around the tip of its tail. Identity orientation points it along the +X axis. 
    // scale.x is the arrow length, scale.y is the arrow width and scale.z is the arrow height.     
    float rollRotVel = 0.0;
    float pitchRotVel = 0.0;
    float yawRotVel = atan2(d_, w_) - M_PI_2;
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollRotVel, pitchRotVel, yaw_ + yawRotVel );
    
    marker.scale.x = ( pow(w_, 2.0) + pow(d_, 2.0) )*yawVel_; // Velocity of the cornerpoint, so scaled with the distance from the center.
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;    
}

std::vector<geo::Vec2f> Rectangle::determineCorners ( float associationDistance )
{
    float dx = 0.5* ( w_ + associationDistance ); // blow up for associations
    float dy = 0.5* ( d_ + associationDistance );

    float ct = cos ( yaw_ );
    float st = sin ( yaw_ );

    geo::Vec2f originCorner ( x_ + ct*-dx + st* dy, y_ + st*-dx + ct*-dy ); // Rotation matrix @ -x, -y
    geo::Vec2f corner1 (      x_ + ct* dx + st* dy, y_ + st* dx + ct*-dy ); // @ +x, -y
    geo::Vec2f corner2 (      x_ + ct* dx - st* dy, y_ + st* dx + ct* dy ); // @ +x, +y
    geo::Vec2f corner3 (      x_ + ct*-dx - st* dy, y_ + st*-dx + ct* dy ); // @ -x, +y

    std::vector< geo::Vec2f > corners;
    corners.push_back ( originCorner );
    corners.push_back ( corner1 );
    corners.push_back ( corner2 );
    corners.push_back ( corner3 );

    return corners;
}

std::vector<geo::Vec2f> Rectangle::determineCenterpointsOfEdges ( )
{
        float rotWidth = yaw_ + M_PI_4;
        float ct = cos ( rotWidth );
        float st = sin ( rotWidth );
        
        float length = 0.5*d_;
        
        geo::Vec2f originCorner ( x_ + ct* length + st* 0, y_ + st* length + ct*0 );
        geo::Vec2f corner1 (      x_ + ct*-length + st* 0, y_ + st*-length + ct*0 );
        
        length = 0.5*w_;
        geo::Vec2f corner2 (      x_ + ct* 0 - st* length, y_ + st* 0 + ct* length );
        geo::Vec2f corner3 (      x_ + ct* 0 - st*-length, y_ + st* 0 + ct* -length );

        std::vector< geo::Vec2f > corners;
        corners.push_back ( originCorner );
        corners.push_back ( corner1 );
        corners.push_back ( corner2 );
        corners.push_back ( corner3 );

        return corners;  
}

bool Rectangle::switchDimensions( float measuredYaw)
{
        return std::fabs( std::fabs( yaw_ - measuredYaw )- M_PI_2 ) < MARGIN_RECTANGLE_INTERCHANGE;
}
void Rectangle::interchangeRectangleFeatures()
{
        float widthOld = w_;
        w_ = d_;
        d_ = widthOld;
        
       yaw_ += M_PI_2;
       
       float P_depthOld = Pdim_ ( 1, 1 );
       Pdim_( 1, 1) = Pdim_( 0, 0);
       Pdim_( 0, 0) = P_depthOld;
}

Eigen::VectorXf Rectangle::setState(float posX, float posY, float posYaw, float xVel, float yVel, float yawVel, float width, float depth)
{
         Eigen::MatrixXf state( 8, 1 );
         state << posX, posY, posYaw, xVel, yVel, yawVel, width, depth;
         
         return state;   
}

bool FeatureProbabilities::setMeasurementProbabilities ( float errorRectangleSquared, float errorCircleSquared, float circleDiameter, float typicalCorridorWidth )
{
    if ( !std::isinf ( errorRectangleSquared ) || !std::isinf ( errorCircleSquared ) )
    {
        float probabilityScaling = 1.0;
        if ( circleDiameter > 0.5*typicalCorridorWidth )
        {
            // Circles with a very large radius could have a smaller error compared to fitting a line. For the type of environment, this is very unlikely.
            // Therefore, the probability of large circles (diameter > typicalCorridorWidth) is reduced in an exponential fashion
            probabilityScaling = std::exp ( -1/ ( 0.5*typicalCorridorWidth ) * ( circleDiameter -0.5*typicalCorridorWidth ) );
        }

        float sum = errorRectangleSquared + errorCircleSquared;
        float pCircle = probabilityScaling * errorRectangleSquared/sum;
        float pRectangle =  1.0 - pCircle;  // Only 2 objects now, so the sum of it equals 1

        pmf_.setProbability ( "Rectangle", pRectangle );
        pmf_.setProbability ( "Circle", pCircle );
        return true;
    }
    else
    {
        // Infinity detected on both of the elements: this is the case when the number of points is too small to do a fit. Equal probability set.
//         pmf_.setProbability ( "Rectangle", 0.5 );
//         pmf_.setProbability ( "Circle", 0.5 );
    
            // TODO if there are enough points for a single fit (probably circle only), is this fit realistic?
            // Acatually, it should be measured if the object which is modelled is realistic by comparing the laser scan with the expected scan based on that object
            
            return false;
    }
}

void FeatureProbabilities::update ( float pRectangle_measured, float pCircle_measured )
{
    pbl::PMF pmf_measured = pmf_;

    pmf_measured.setProbability ( "Rectangle", pRectangle_measured );
    pmf_measured.setProbability ( "Circle", pCircle_measured );

    pmf_.update ( pmf_measured );
}

void FeatureProbabilities::update ( FeatureProbabilities& featureProbabilities_in )
{
    this->pmf_.update ( featureProbabilities_in.pmf_ );
}

void FeatureProperties::updateCircleFeatures ( Eigen::MatrixXf Q_k, Eigen::MatrixXf R_k, Eigen::MatrixXf z_k, float dt )
// z = observation, dt is the time difference between the latest update and the new measurement
{
        unsigned int x_PosVelRef = 0, y_PosVelRef = 1, xVel_PosVelRef = 2, yVel_PosVelRef = 3;
        unsigned int r_dimRef = 0;      
        unsigned int x_zRef = 0, y_zRef = 1, radius_zRef = 2;
        
        Eigen::MatrixXf F_PosVel ( 4, 4 );
        F_PosVel << 1.0, 0.0, dt,  0.0,  // x 
                    0.0, 1.0, 0.0, dt,   // y 
                    0.0, 0.0, 1.0, 0.0,  // x vel 
                    0.0, 0.0, 0.0, 1.0;  // y vel
                
        Eigen::MatrixXf Fdim ( 1, 1 );    
        Fdim <<     1.0;               // radius
                
        Eigen::MatrixXf H_PosVel ( 2, 4 );
        H_PosVel << 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0;
         
        Eigen::MatrixXf Hdim ( 1, 1 );
        Hdim.setIdentity( Hdim.rows(), Hdim.cols() );
        
        // First, update the dimensions
        Eigen::MatrixXf x_k_1_k_1_dim ( 1, 1 ), z_k_dim( 1, 1 );
        x_k_1_k_1_dim << circle_.get_radius();
        z_k_dim << z_k( radius_zRef );
        Eigen::MatrixXf Pdim = circle_.get_Pdim();
        
        Eigen::MatrixXf x_k_k_dim = kalmanUpdate(Fdim, Hdim, &Pdim, x_k_1_k_1_dim, z_k_dim, Q_k.block<1, 1>( 4, 4 ), R_k.block<1, 1>( 2, 2 ) );
        
        // After the position update for changed dimensions, update the dimensions
        Eigen::MatrixXf P_PosVel = circle_.get_P();
        Eigen::MatrixXf x_k_1_k_1_PosVel( 4, 1 ), z_k_posVel( 2, 1 );
        x_k_1_k_1_PosVel << circle_.get_x(), circle_.get_y(), circle_.get_xVel(), circle_.get_yVel();
        z_k_posVel << z_k ( x_zRef ), z_k ( y_zRef );
    
        Eigen::MatrixXf x_k_k_PosVel = kalmanUpdate(F_PosVel, H_PosVel, &P_PosVel, x_k_1_k_1_PosVel, z_k_posVel, Q_k.block<4, 4>( 0, 0 ), R_k.block<2, 2>( 0, 0 ) );         
        
        circle_.set_x ( x_k_k_PosVel ( x_PosVelRef ) );
        circle_.set_y ( x_k_k_PosVel ( y_PosVelRef ) );
        circle_.set_xVel ( x_k_k_PosVel ( xVel_PosVelRef ) );
        circle_.set_yVel ( x_k_k_PosVel ( yVel_PosVelRef ) );
        circle_.set_radius ( x_k_k_dim( r_dimRef ) );

        circle_.set_P ( P_PosVel );
        circle_.set_Pdim ( Pdim );
}

void FeatureProperties::updateRectangleFeatures ( Eigen::MatrixXf Q_k, Eigen::MatrixXf R_k, Eigen::VectorXf z_k, float dt, const geo::Pose3D& sensor_pose )
{
        // z = observation, dt is the time difference between the latest update and the new measurement
        // 2 stages: first determine the updated width en depth, then use this difference to update the position first in order to prevent ghost-velocities. 
        
        // conversion for general state to state for (1) the position and velocity state and (2) the dimension state
        unsigned int x_PosVelRef = 0, y_PosVelRef = 1, yaw_PosVelRef = 2, xVel_PosVelRef = 3, yVel_PosVelRef = 4, yawVel_PosVelRef = 5;
        unsigned int width_dimRef = 0, depth_dimRef = 1;
        unsigned int x_zRef = 0, y_zRef = 1, yaw_zRef = 2, width_zRef = 3, depth_zRef = 4;
        
        Eigen::MatrixXf F_PosVel ( 6, 6 );
        F_PosVel << 1.0, 0.0, 0.0, dt,  0.0, 0.0, // x 
                    0.0, 1.0, 0.0, 0.0, dt,  0.0, // y 
                    0.0, 0.0, 1.0, 0.0, 0.0, dt,  // orientation
                    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, // x vel 
                    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, // y vel 
                    0.0, 0.0, 0.0, 0.0, 0.0, 1.0; // rotational vel
                
        Eigen::MatrixXf Fdim ( 2, 2 );    
        Fdim <<     1.0, 0.0,               // width
                    0.0, 1.0;               // length
                
        Eigen::MatrixXf H_PosVel ( 3, 6 );
        H_PosVel << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
         
        Eigen::MatrixXf Hdim ( 2, 2 );
        Hdim.setIdentity( Hdim.rows(), Hdim.cols() ); 
                
        unwrap( &z_k( yaw_zRef ), rectangle_.get_yaw(), M_PI );

        if( rectangle_.switchDimensions( z_k( yaw_zRef ) ) )
        {
                rectangle_.interchangeRectangleFeatures( );
                unwrap( &z_k( yaw_zRef ), rectangle_.get_yaw(), M_PI ); 
        }
        
        Eigen::MatrixXf Pdim = rectangle_.get_Pdim();
        Eigen::MatrixXf x_k_1_k_1_dim( 2, 1 ), z_k_dim( 2, 1 );
        x_k_1_k_1_dim << rectangle_.get_w(), rectangle_.get_d();
        z_k_dim << z_k( width_zRef ), z_k( depth_zRef );
        
        Eigen::MatrixXf x_k_k_dim = kalmanUpdate(Fdim, Hdim, &Pdim, x_k_1_k_1_dim, z_k_dim, Q_k.block<2, 2>( 6, 6 ), R_k.block<2, 2>( 3, 3 ) );
        
        // Correct position for changes caused by a change in dimension
//         float deltaWidth = x_k_k_dim( width_dimRef ) - x_k_1_k_1_dim ( width_dimRef );
//         float deltaDepth = x_k_k_dim( depth_dimRef ) - x_k_1_k_1_dim ( depth_dimRef );
        
        float deltaWidth = x_k_1_k_1_dim ( width_dimRef ) - x_k_k_dim( width_dimRef );
        float deltaDepth = x_k_1_k_1_dim ( depth_dimRef ) - x_k_k_dim( depth_dimRef );
      /* 
        float deltaX_dim, deltaY_dim;
        correctPosForDimDiff(deltaWidth, deltaDepth, &deltaX_dim, &deltaY_dim, dt );
        
        // What would the previous position of the measurement be given the velocities already estimated?
        float ZPrevX = z_k( x_zRef ) - rectangle_.get_xVel()*dt;
        float ZPrevY = z_k( y_zRef ) - rectangle_.get_yVel()*dt;
        
        // Determine the direction of the correction based on an estimation of the position of the measurement at the previous timestamp
        int signX = ( std::fabs( rectangle_.get_x() + deltaX_dim - ZPrevX ) < std::fabs( rectangle_.get_x() - deltaX_dim - ZPrevX ) ) ? 1 : -1;
        int signY = ( std::fabs( rectangle_.get_y() + deltaY_dim - ZPrevY ) < std::fabs( rectangle_.get_y() - deltaY_dim - ZPrevY ) ) ? 1 : -1;
        rectangle_.set_x ( rectangle_.get_x() + deltaX_dim );
        rectangle_.set_y ( rectangle_.get_y() + deltaY_dim );
       */
        
       float deltaX = 0.0, deltaY = 0.0;
       correctForDimensions( deltaWidth, deltaDepth, &deltaX, &deltaY, sensor_pose, dt );
       
        std::cout << "Delta x dim = " << deltaX << "Delta y dim = " << deltaY << std::endl;
        
        rectangle_.set_x ( rectangle_.get_x() + deltaX );
        rectangle_.set_y ( rectangle_.get_y() + deltaY );
        
        rectangle_.set_w ( x_k_k_dim ( width_dimRef ) );
        rectangle_.set_d ( x_k_k_dim ( depth_dimRef ) );
        
        // Correct measured position caused by differences in modelled and measured dimensions
        deltaWidth = rectangle_.get_w() - z_k ( width_zRef );
        deltaDepth = rectangle_.get_d() - z_k ( depth_zRef );
        
        correctForDimensions( deltaWidth, deltaDepth, &z_k( x_zRef ), &z_k( y_zRef ), sensor_pose, dt );
        
        std::cout << "Delta x meas dim = " << deltaWidth << " Delta y dim = " << deltaDepth << std::endl;
//         std::cout << "signWidth = " << signWidth << " signWidth = " << signDepth << std::endl;
//         std::cout << "deltaX_VelWidth = " << deltaX_VelWidth << " deltaX_VelWidth = " << deltaX_VelWidth << std::endl;
//         std::cout << "deltaX_VelDepth = " << deltaX_VelDepth << " deltaY_VelDepth = " << deltaY_VelDepth << std::endl;
        
        /*
        float deltaX_VelWidth, deltaY_VelWidth, deltaX_VelDepth, deltaY_VelDepth;
        correctPosForDimDiff(deltaWidth, 0, &deltaX_VelWidth, &deltaY_VelWidth, dt, z_k( yaw_zRef ) );
        correctPosForDimDiff(0, deltaDepth, &deltaX_VelDepth, &deltaY_VelDepth, dt, z_k( yaw_zRef ) );
        
        // Strategy previously tested: do a prediction of the position and check which (correction + measurement) is closest to the this prediction
        // Problem: wrong estimation of velocity can lead to measurements being corrected into the wrong direction, reflecting a position which can not be measured! (i.e., the position is closer
        // to the sensor than the measurement obtained )        
        float distPosWidth2 = pow( sensor_pose.getOrigin().getX() - ( z_k( x_zRef ) + deltaX_VelWidth), 2.0 ) + pow( sensor_pose.getOrigin().getY() - ( z_k( y_zRef ) + deltaY_VelWidth), 2.0 );
        float distNegWidth2 = pow( sensor_pose.getOrigin().getX() - ( z_k( x_zRef ) - deltaX_VelWidth), 2.0 ) + pow( sensor_pose.getOrigin().getY() - ( z_k( y_zRef ) - deltaY_VelWidth), 2.0 );
        
        float distPosDepth2 = pow( sensor_pose.getOrigin().getX() - ( z_k( x_zRef ) + deltaX_VelDepth), 2.0 ) + pow( sensor_pose.getOrigin().getY() - ( z_k( y_zRef ) + deltaY_VelDepth), 2.0 );
        float distNegDepth2 = pow( sensor_pose.getOrigin().getX() - ( z_k( x_zRef ) - deltaX_VelDepth), 2.0 ) + pow( sensor_pose.getOrigin().getY() - ( z_k( y_zRef ) - deltaY_VelDepth), 2.0 );
        
        bool largerDistanceDesiredWidth = deltaWidth > 0 ? 1 : 0 ;
        bool largerDistanceDesiredDepth = deltaDepth > 0 ? 1 : 0 ;
        
        int signWidth =  largerDistanceDesiredWidth && distPosWidth2 > distNegWidth2 || !largerDistanceDesiredWidth && distPosWidth2 <  distNegWidth2 ? 1 : -1;
        int signDepth =  largerDistanceDesiredDepth && distPosDepth2 > distNegDepth2 || !largerDistanceDesiredDepth && distPosDepth2 <  distNegDepth2 ? 1 : -1;
        
        z_k ( x_zRef ) += signWidth*deltaX_VelWidth + signDepth*deltaX_VelDepth;
        z_k ( y_zRef ) += signWidth*deltaY_VelWidth + signDepth*deltaY_VelDepth;
        /*
        /*std::cout << "Delta x meas dim = " << signWidth*deltaX_VelWidth + signDepth*deltaX_VelDepth << " Delta y dim = " << signWidth*deltaY_VelWidth + signDepth*deltaY_VelDepth<< std::endl;
        std::cout << "signWidth = " << signWidth << " signWidth = " << signDepth << std::endl;
        std::cout << "deltaX_VelWidth = " << deltaX_VelWidth << " deltaX_VelWidth = " << deltaX_VelWidth << std::endl;
        std::cout << "deltaX_VelDepth = " << deltaX_VelDepth << " deltaY_VelDepth = " << deltaY_VelDepth << std::endl;
        */
        
        Eigen::MatrixXf P = rectangle_.get_P();
        Eigen::MatrixXf x_k_1_k_1_PosVel( 6, 1 ), z_k_posVel( 3, 1 );
        x_k_1_k_1_PosVel << rectangle_.get_x(), rectangle_.get_y(), rectangle_.get_yaw(), rectangle_.get_xVel(), rectangle_.get_yVel(), rectangle_.get_yawVel();
        z_k_posVel <<       z_k ( x_zRef ),     z_k ( y_zRef ),     z_k ( yaw_zRef );
        
        Eigen::MatrixXf x_k_k_PosVel = kalmanUpdate(F_PosVel, H_PosVel, &P, x_k_1_k_1_PosVel, z_k_posVel, Q_k.block< 6, 6 >( 0, 0 ), R_k.block< 3, 3 >( 0, 0 ));
        
        rectangle_.set_x ( x_k_k_PosVel ( x_PosVelRef ) );
        rectangle_.set_y ( x_k_k_PosVel ( y_PosVelRef ) );
        rectangle_.set_yaw ( x_k_k_PosVel ( yaw_PosVelRef ) );
        rectangle_.set_xVel ( x_k_k_PosVel ( xVel_PosVelRef ) );
        rectangle_.set_yVel ( x_k_k_PosVel ( yVel_PosVelRef ) );
        rectangle_.set_yawVel ( x_k_k_PosVel ( yawVel_PosVelRef ) );

        rectangle_.set_P ( P );
        rectangle_.set_Pdim( Pdim );
}

void FeatureProperties::correctForDimensions( float deltaWidth, float deltaDepth, float* xMeasured, float* yMeasured, const geo::Pose3D& sensor_pose, float dt )
{
        float deltaX_Width, deltaY_Width, deltaX_Depth, deltaY_Depth;
        correctPosForDimDiff(deltaWidth, 0, &deltaX_Width, &deltaY_Width, dt );
        correctPosForDimDiff(0, deltaDepth, &deltaX_Depth, &deltaY_Depth, dt );
        
        // Strategy previously tested: do a prediction of the position and check which (correction + measurement) is closest to the this prediction
        // Problem: wrong estimation of velocity can lead to measurements being corrected into the wrong direction, reflecting a position which can not be measured! (i.e., the position is closer
        // to the sensor than the measurement obtained )        
        float distPosWidth2 = pow( sensor_pose.getOrigin().getX() - ( *xMeasured + deltaX_Width), 2.0 ) + pow( sensor_pose.getOrigin().getY() - ( *yMeasured + deltaY_Width), 2.0 );
        float distNegWidth2 = pow( sensor_pose.getOrigin().getX() - ( *xMeasured - deltaX_Width), 2.0 ) + pow( sensor_pose.getOrigin().getY() - ( *yMeasured - deltaY_Width), 2.0 );
        
        float distPosDepth2 = pow( sensor_pose.getOrigin().getX() - ( *xMeasured + deltaX_Depth), 2.0 ) + pow( sensor_pose.getOrigin().getY() - ( *yMeasured + deltaY_Depth), 2.0 );
        float distNegDepth2 = pow( sensor_pose.getOrigin().getX() - ( *xMeasured - deltaX_Depth), 2.0 ) + pow( sensor_pose.getOrigin().getY() - ( *yMeasured - deltaY_Depth), 2.0 );
        
        bool largerDistanceDesiredWidth = deltaWidth > 0 ? 1 : 0 ;
        bool largerDistanceDesiredDepth = deltaDepth > 0 ? 1 : 0 ;
        
        int signWidth =  largerDistanceDesiredWidth && distPosWidth2 > distNegWidth2 || !largerDistanceDesiredWidth && distPosWidth2 <  distNegWidth2 ? 1 : -1;
        int signDepth =  largerDistanceDesiredDepth && distPosDepth2 > distNegDepth2 || !largerDistanceDesiredDepth && distPosDepth2 <  distNegDepth2 ? 1 : -1;
        
        *xMeasured += signWidth*deltaX_Width + signDepth*deltaX_Depth;
        *yMeasured += signWidth*deltaY_Width + signDepth*deltaY_Depth;
}

void FeatureProperties::correctPosForDimDiff(float deltaWidth, float deltaDepth, float *deltaX, float *deltaY, float dt)
{
        float thetaPred = rectangle_.get_yaw() + dt*rectangle_.get_yawVel();
        
        float pred_st = std::sin( thetaPred );
        float pred_mct = std::cos( thetaPred );
                
        // check in which direction the measured center-point falls in order to determine to right direction of correction in both x and y
        float rotatedX = deltaWidth * pred_mct - deltaDepth * pred_st;
        float rotatedY = deltaWidth * pred_st + deltaDepth * pred_mct;

        *deltaX = 0.5*rotatedX;
        *deltaY =  0.5*rotatedY;
}

void FeatureProperties::printProperties()
{
        rectangle_.printProperties();
        circle_.printProperties();
}

}

namespace convex_hull
{
// ----------------------------------------------------------------------------------------------------

void create ( const std::vector<geo::Vec2f>& points, float z_min, float z_max, ConvexHull& chull, geo::Pose3D& pose )
{
    cv::Mat_<cv::Vec2f> points_2d ( 1, points.size() );
    for ( unsigned int i = 0; i < points.size(); ++i ) {
        points_2d.at<cv::Vec2f> ( i ) = cv::Vec2f ( points[i].x, points[i].y );
    }

    pose = geo::Pose3D::identity();

    pose.t.z = ( z_min + z_max ) / 2;

    std::vector<int> chull_indices;
    cv::convexHull ( points_2d, chull_indices );

    chull.z_min = z_min - pose.t.z;
    chull.z_max = z_max - pose.t.z;

    geo::Vec2f xy_min ( 1e9, 1e9 );
    geo::Vec2f xy_max ( -1e9, -1e9 );

    chull.points.clear();
    for ( unsigned int i = 0; i < chull_indices.size(); ++i ) {
        const cv::Vec2f& p_cv = points_2d.at<cv::Vec2f> ( chull_indices[i] );
        geo::Vec2f p ( p_cv[0], p_cv[1] );

        chull.points.push_back ( p );

        xy_min.x = std::min ( xy_min.x, p.x );
        xy_min.y = std::min ( xy_min.y, p.y );

        xy_max.x = std::max ( xy_max.x, p.x );
        xy_max.y = std::max ( xy_max.y, p.y );
    }

    // Average segment position
    pose.t.x = ( xy_min.x + xy_max.x ) / 2;
    pose.t.y = ( xy_min.y + xy_max.y ) / 2;

    // Move all points to the pose frame
    for ( unsigned int i = 0; i < chull.points.size(); ++i ) {
        geo::Vec2f& p = chull.points[i];
        p.x -= pose.t.x;
        p.y -= pose.t.y;
    }

    // Calculate normals and edges
    convex_hull::calculateEdgesAndNormals ( chull );

    // Calculate area
    calculateArea ( chull );
}

// ----------------------------------------------------------------------------------------------------

void createAbsolute ( const std::vector<geo::Vec2f>& points, float z_min, float z_max, ConvexHull& c )
{
    cv::Mat_<cv::Vec2f> points_2d ( 1, points.size() );
    for ( unsigned int i = 0; i < points.size(); ++i ) {
        points_2d.at<cv::Vec2f> ( i ) = cv::Vec2f ( points[i].x, points[i].y );
    }

    c.z_min = z_min;
    c.z_max = z_max;

    std::vector<int> chull_indices;
    cv::convexHull ( points_2d, chull_indices );

    c.points.resize ( chull_indices.size() );
    for ( unsigned int i = 0; i < chull_indices.size(); ++i ) {
        const cv::Vec2f& p_cv = points_2d.at<cv::Vec2f> ( chull_indices[i] );
        c.points[i] = geo::Vec2f ( p_cv[0], p_cv[1] );
    }

    // Calculate normals and edges
    convex_hull::calculateEdgesAndNormals ( c );

    // Calculate area
    calculateArea ( c );
}

// ----------------------------------------------------------------------------------------------------

void calculateEdgesAndNormals ( ConvexHull& c )
{
    c.edges.resize ( c.points.size() );
    c.normals.resize ( c.points.size() );

    for ( unsigned int i = 0; i < c.points.size(); ++i ) {
        unsigned int j = ( i + 1 ) % c.points.size();

        const geo::Vec2f& p1 = c.points[i];
        const geo::Vec2f& p2 = c.points[j];

        // Calculate edge
        geo::Vec2f e = p2 - p1;
        c.edges[i] = e;

        // Calculate normal
        c.normals[i] = geo::Vec2f ( e.y, -e.x ).normalized();
    }
}

// ----------------------------------------------------------------------------------------------------

bool collide ( const ConvexHull& c1, const geo::Vector3& pos1,
               const ConvexHull& c2, const geo::Vector3& pos2,
               float xy_padding, float z_padding )
{
    if ( c1.points.size() < 3 || c2.points.size() < 3 ) {
        return false;
    }

    float z_diff = pos2.z - pos1.z;

    if ( c1.z_max < ( c2.z_min + z_diff - 2 * z_padding ) || c2.z_max < ( c1.z_min - z_diff - 2 * z_padding ) ) {
        return false;
    }

    geo::Vec2f pos_diff ( pos2.x - pos1.x, pos2.y - pos1.y );

    for ( unsigned int i = 0; i < c1.points.size(); ++i ) {
        const geo::Vec2f& p1 = c1.points[i];
        const geo::Vec2f& n = c1.normals[i];

        // Calculate min and max projection of c1
        float min1 = n.dot ( c1.points[0] - p1 );
        float max1 = min1;
        for ( unsigned int k = 1; k < c1.points.size(); ++k ) {
            // Calculate projection
            float p = n.dot ( c1.points[k] - p1 );
            min1 = std::min ( min1, p );
            max1 = std::max ( max1, p );
        }

        // Apply padding to both sides
        min1 -= xy_padding;
        max1 += xy_padding;

        // Calculate p1 in c2's frame
        geo::Vec2f p1_c2 = p1 - pos_diff;

        // If this bool stays true, there is definitely no collision
        bool no_collision = true;

        // True if projected points are found below c1's bounds
        bool below = false;

        // True if projected points are found above c1's bounds
        bool above = false;

        // Check if c2's points overlap with c1's bounds
        for ( unsigned int k = 0; k < c2.points.size(); ++k ) {
            // Calculate projection on p1's normal
            float p = n.dot ( c2.points[k] - p1_c2 );

            below = below || ( p < max1 );
            above = above || ( p > min1 );

            if ( below && above ) {
                // There is overlap with c1's bound, so we may have a collision
                no_collision = false;
                break;
            }
        }

        if ( no_collision )
            // definitely no collision
        {
            return false;
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

void calculateArea ( ConvexHull& c )
{
    c.area = 0;
    for ( unsigned int i = 0; i < c.points.size(); ++i ) {
        unsigned int j = ( i + 1 ) % c.points.size();

        const geo::Vec2f& p1 = c.points[i];
        const geo::Vec2f& p2 = c.points[j];

        c.area += 0.5 * ( p1.x * p2.y - p2.x * p1.y );
    }
}

// ----------------------------------------------------------------------------------------------------

}

}
