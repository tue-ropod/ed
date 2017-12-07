#include "ed/convex_hull_calc.h"

#include <opencv2/imgproc/imgproc.hpp>

namespace ed
{

namespace tracking
{

//Fast Line, Arc/Circle and Leg Detection from Laser Scan Data in a Player Driver: http://miarn.sourceforge.net/pdf/a1738b.pdf

void fitCircle ( const std::vector<geo::Vec2f>& points, ed::tracking::Circle* cirlce, geo::Pose3D& pose )
{

    // according to https://dtcenter.org/met/users/docs/write_ups/circle_fit.pdf
    float x_avg = 0.0, y_avg = 0.0;
    for ( unsigned int i = 0; i < points.size(); ++i ) {
        std::cout << "Points = " << points[i].x << ", " << points[i].y << std::endl;
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
    float R = std::sqrt ( alpha );
    std::cout << "fitter: x_, y_, R_" << xc << ", " << yc << ", " << R << std::endl;
    cirlce->setValues ( xc, yc,  R );
}

void Circle::setValues ( float x, float y, float R )
{
    this->x_ = x;
    this->y_ = y;
    this->R_ = R;
}

void Circle::setMarker ( visualization_msgs::Marker& marker , unsigned int ID )
{
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = ID;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = 1;

    //std::cout << "MArker: x_, y_, R_" << x_ << ", " << y_ << ", " << R_ << std::endl;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2*R_;
    marker.scale.y = 2*R_;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.lifetime = ros::Duration ( TIMEOUT_TIME );
}

void fitRectangle ( const std::vector<geo::Vec2f>& points, ed::tracking::Rectangle* rectangle, geo::Pose3D& pose )
{
    // First determine the lines
    float angle, angle1, angle2;
    geo::Vec2f centroid, centroid1, centroid2;
    unsigned int max_error_index, max_error_index1, max_error_index2;
    
    bool fit_rectangle = false;
    std::cout << "Start fitLine" << std::endl;
    float max_error = fitLine(points, angle, centroid, max_error_index);
    if( max_error > MAX_LINE_ERROR){
      fit_rectangle = true;
    }
    
    std::cout << "Max error = " << max_error << std::endl;

  
   if( fit_rectangle ){ // split data at max error and repeat for both left and right lines
     
     std::cout << "Try to fit rectangle in data" << std::endl;
     
     const std::vector<geo::Vec2f> points_low( points.begin(), points.begin() + max_error_index);
     const std::vector<geo::Vec2f> points_hi( points.begin()+ max_error_index, points.end() );
     
     std::cout << "Try to fit 2 lines" << std::endl;
     float max_error1 = fitLine(points_low, angle1, centroid1, max_error_index1);
     float max_error2 = fitLine(points_hi, angle2, centroid2, max_error_index2);
     
     if(max_error1 > MAX_LINE_ERROR || max_error2 > MAX_LINE_ERROR){
       std::cout << "WARNING: more line segments needed (is quare assumption valid?)" << std::endl;
     }
     
     std::cout << "Lines fitted" << std::endl;
     float center_x = (centroid1.x + centroid2.x )*0.5;
     float center_y = (centroid1.y + centroid2.y )*0.5;
     
     //determine width and height
     float dx = points_low[points_low.size()-1].x-points_low[0].x;
     float dy = points_low[points_low.size()-1].y-points_low[0].y;
     float width = sqrt(dx*dx+dy*dy);
     float theta = atan(dy/dx);
     
     dx = points_hi[points_hi.size()-1].x-points_hi[0].x;
     dy = points_hi[points_hi.size()-1].y-points_hi[0].y;
     float height = sqrt(dx*dx+dy*dy);
     
      std::cout << "Set values of rectangle" << std::endl;
     rectangle->setValues(center_x, center_y, width, height, theta);
   } else 
   {
     
     std::cout << "Single line considered" << std::endl;
     float center_x = centroid.x;
     float center_y = centroid.y;
     
     float dx = points[points.size()-1].x-points[0].x;
     float dy = points[points.size()-1].y-points[0].y;
     float width = sqrt(dx*dx+dy*dy);
     float theta = atan(dy/dx);
     
     std::cout << "Beginpoint = " << points[0].x << ", Endpoint = " << points[points.size()-1].x << std::endl;
     
     std::cout <<"dx, dy, width = " << dx  << ", " << dy  << ", " << width  << std::endl;
     
     float height = 0.1; // TODO: magic number
     
     std::cout << "Set values of rectangle (using a single line)" << std::endl;
     rectangle->setValues(center_x, center_y, width, height, theta);
   }
   
   
}

float fitLine ( const std::vector<geo::Vec2f>& points, float& angle, geo::Vec2f& centroid, unsigned int& index)
{
  
    float x_sum = 0.0, y_sum = 0.0;
    for ( unsigned int i = 0; i < points.size(); ++i ) {
        //std::cout << "Points = " << points[i].x << ", " << points[i].y << std::endl;
        x_sum += points[i].x;
        y_sum += points[i].y;
    }
    
    centroid.x  = x_sum / points.size();
    centroid.y  = y_sum / points.size();
    
    std::vector<geo::Vec2f> points_corrected = points;
    float sum_xy = 0.0, sum_squared_diff = 0.0;
    for ( unsigned int i = 0; i < points.size(); ++i ) {
        std::cout << "Points = " << points[i].x << ", " << points[i].y << std::endl;
        points_corrected[i].x -= centroid.x;
	points_corrected[i].y -= centroid.y;
	
	sum_xy += points_corrected[i].x * points_corrected[i].y;
	sum_squared_diff += points_corrected[i].x * points_corrected[i].x - points_corrected[i].y * points_corrected[i].y;
    }
    
    std::cout << "fitLine 1" << std::endl;
    
    float A = sum_squared_diff/sum_xy;
    float angle1 = -A - sqrt(0.5*(A*A+4));
    float angle2 = -A + sqrt(0.5*(A*A+4));
    
    float error1 = 0.0, error2 = 0.0, max_error1 = 0.0, max_error2 = 0.0, max_error, error; // max error of line with best fit
    unsigned int error_index1, error_index2;
    for ( unsigned int i = 0; i < points.size(); ++i ) {
      // Distance of each point to line
        error = fabs ( angle1*points_corrected[i].x-points_corrected[i].y ) /sqrt ( angle1*angle1+1 );
        error1 += error;

        if ( error > max_error1 ) {
            max_error1 = error;
	    error_index1 = i;
        }

        error = fabs ( angle2*points_corrected[i].x-points_corrected[i].y ) /sqrt ( angle2*angle2+1 );
        error2 += error;

        if ( error > max_error2 ) {
            max_error2 = error;
	    error_index2 = i;
        }

        if ( error1 < error2 ) {
            angle = angle1;
            max_error = max_error1;
	    index = error_index1;
        } else {
            angle = angle2;
            max_error = max_error2;
	    index = error_index2;
        }

        
    }
    
    std::cout << "fitLine 2" << std::endl;
    return max_error;
}

void Rectangle::setValues ( float x, float y, float w, float h, float theta )
{
  //std::cout << "Going to set values x, y, w, g, theta = " << x << ", " << y << ", " << w << ", " << h  << ", " << theta << std::endl;

    this->x_ = x;
    std::cout << "x set" << std::endl;
    
    this->y_ = y;
    std::cout << "y set" << std::endl;
    
    this->w_ = w;
    std::cout << "w set" << std::endl;
    
    this->h_ = h;
    std::cout << "h set" << std::endl;
    
    this->theta_ = theta;
    
    //std::cout << "theta set -> values finished" << std::endl;
}

void Rectangle::setMarker ( visualization_msgs::Marker& marker, unsigned int ID )
{
    // How to incorporate the rotation? -> should follow from the lines
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = ID;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = 1;

    //std::cout << "MArker: x_, y_, R_" << x_ << ", " << y_ << ", " << R_ << std::endl;

    marker.pose.orientation.x = cos(theta_/2);
    marker.pose.orientation.y = sin(theta_/2);
    marker.pose.orientation.z = sin(theta_/2);
    marker.pose.orientation.w = sin(theta_/2);
    marker.scale.x = w_;
    marker.scale.y = h_;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.lifetime = ros::Duration ( TIMEOUT_TIME );
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
