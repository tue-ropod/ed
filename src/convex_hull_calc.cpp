#include "ed/convex_hull_calc.h"

#include <opencv2/imgproc/imgproc.hpp>

namespace ed
{

namespace tracking
{

FITTINGMETHOD determineCase ( std::vector<geo::Vec2f>& points, unsigned int* cornerIndex, std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high )
{

    bool includeCorner = findPossibleCorner ( points, *cornerIndex );

    *it_low = points.begin();
    *it_high = points.end();
	    
   /* if ( !splitData ) {
        return LINE;
    } else {
        // first, determine the inscribed radius of all points
        float mean_iav, std_dev_iav;
        unsigned int min_counter;

        std::vector<float> iav = ed::tracking::inscribedRadius ( points, &mean_iav, &std_dev_iav, &min_counter );

        bool criterion1 = mean_iav >= MIN_MEAN_ANGLE_CIRCLE_RAD;
        bool criterion2 = mean_iav <= MAX_MEAN_ANGLE_CIRCLE_RAD;
        bool criterion3 = std_dev_iav >= MIN_DEV_CIRCLE_RAD;
        bool criterion4 = std_dev_iav <= MAX_DEV_CIRCLE_RAD;
	
	std::cout << "[mean_iav std_dev_iav] = [" << mean_iav << ", " << std_dev_iav << "]" << std::endl;

        if ( criterion1 && criterion2 && criterion3 && criterion4 ) {
	  std::cout << "Case = CIRCLE" << std::endl;
            return CIRCLE;
        } else { // check if there are enough points to do an evaluation of a rectangle on the complete data, otherwise remove the data and go for the line-fit
*/
            //std::cout << "Max error criterion not met, or (1) remove points and try to fit line again, or (2) split points and try to fit square" << std::endl;

            
            
            // We do not necessarily need to remove the points, only when we want to fit a line based on square data -> make new list with shorter length of points
            
          if(includeCorner) {
            unsigned int nPointsLow = *cornerIndex;
            unsigned int nPointsHigh = points.size() - *cornerIndex + 1; // +1 because the point with max error is considered as the corner point and belongs to both lines
            unsigned int remainingSize = points.size();

            std::cout << "[nPointsLow nPointsHigh] = [" << nPointsLow << ", " << nPointsHigh << "]"  << std::endl;

            bool fitSingleline = false;
            bool pointsRemoved = false;
            if ( nPointsLow < MIN_POINTS_LINEFIT ) { // Part of section too smal -> remove it from the data which are analyzed and try to fit line again
                //points.erase ( points.begin(), points.begin() + nPointsLow - 1 );
	      *it_low += nPointsLow - 1;
	      remainingSize -= nPointsLow + 1;
	      pointsRemoved = true;
            }

            if ( nPointsHigh < MIN_POINTS_LINEFIT ) {
                //points.erase ( points.end() - ( int )  nPointsHigh + 1,points.end() );
	      *it_high -= nPointsHigh + 1;
	      remainingSize -= nPointsHigh + 1;
	      pointsRemoved = true;
            }
            
            std::cout << "After removal of points: points size = " << points.size() << " remainingSize = " << remainingSize << std::endl;

            if ( pointsRemoved && remainingSize < MIN_POINTS_LINEFIT ) {
                *cornerIndex = std::numeric_limits<unsigned int>::quiet_NaN();
                return NONE;
            } else if ( pointsRemoved && remainingSize >= MIN_POINTS_LINEFIT ) {
                *cornerIndex = std::numeric_limits<unsigned int>::quiet_NaN();
		 std::cout << "Case = LINE" << std::endl;
                return LINE;
            } else  { // we dit not remove points and a corner is present
	      std::cout << "Case = RECTANGLE" << std::endl;
                return RECTANGLE;
	    }
            
	  } else {
	       std::cout << "Case = LINE" << std::endl;
                return LINE;
	  }
            
            
            return NONE;
        //}
    //}
}

float fitObject ( std::vector<geo::Vec2f>& points, geo::Pose3D& pose, int FITTINGMETHOD,  unsigned int* cornerIndex, ed::tracking::Rectangle* rectangle, ed::tracking::Circle* circle, unsigned int* ID, visualization_msgs::Marker* marker,  std::vector<geo::Vec2f>::iterator* it_low, std::vector<geo::Vec2f>::iterator* it_high )
{

    switch ( FITTINGMETHOD ) {
      case NONE: {
	return std::numeric_limits<float>::infinity();
      }
    case LINE: {
        // fit line, determine length using least-squares method to determine theta
      
      std::cout << "Try to fit line in fitObject-function " << std::endl;

        Eigen::VectorXd beta_hat ( 2 );
	float mean_error = fitLine ( points, beta_hat, *it_low, *it_high ) ;

        float theta = atan2 ( beta_hat ( 1 ), 1 );

        float reference_x = points[0].x;
        float reference_y = beta_hat ( 1 ) * reference_x + beta_hat ( 0 );

        float x_end = points[points.size() - 1].x;
	float y_end = beta_hat ( 1 ) * x_end + beta_hat ( 0 );
        float dx = reference_x - x_end;
        float dy = reference_y -  y_end;
        float width = sqrt ( dx*dx+dy*dy );
	
	//std::cout << "Width after Line-fit  = " << width << std::endl;
	
	std::cout << "[reference_x, reference_y, dx, dy, width] = [" << reference_x << ", " << reference_y << ", " << dx << ", " <<  dy  << ", " << width << "]" << std::endl;
	std::cout << "Original [reference_y, dy] = [" <<  points[points.size() - 1].y << ", " << points[points.size() - 1].y - points[0].y << "]" << std::endl;

        rectangle->setValues ( reference_x, reference_y, width, ARBITRARY_DEPTH, ARBITRARY_HEIGHT, theta );

        return mean_error;
    }

    case CIRCLE: {
      
      float mean_error = fitCircle ( points, circle, pose );      
      return mean_error;
      
    }
    case RECTANGLE: {
      float mean_error = fitRectangle ( points, rectangle, pose , cornerIndex );
      return mean_error;
    }
    
    //
    }
    return false; // end reached without doing something

}

std::vector<float> inscribedRadius ( std::vector<geo::Vec2f>& points, float* mean, float* std_dev, unsigned int* arg_min )
{

    Point A ( points[0].x, points[0].y );
    Point C ( points[points.size() - 1].x, points[points.size() - 1].y );

    std::vector<float> iav ( points.size() - 2 );
    for ( unsigned int ii = 0; ii < iav.size(); ++ii ) {
        int jj = ii + 1;

        Point B ( points[jj].x, points[jj].y );

        float c2 = pow ( ( A.x-B.x ), 2.0 ) + pow ( ( A.y-B.y ), 2.0 );
        float b2 = pow ( ( C.x-A.x ), 2.0 ) + pow ( ( C.y-A.y ), 2.0 );
        float a2 = pow ( ( C.x-B.x ), 2.0 ) + pow ( ( C.y-B.y ), 2.0 );

        iav[ii] = acos ( - ( b2-a2-c2 ) / ( 2*sqrt ( a2*c2 ) ) );


        if ( iav[ii] != iav[ii] ) {
            //std::cout << "[a2, b2, c2] = " << a2 << ", " << b2 << ", " << c2;
            //std::cout << " [ax, bx, cx] = " << A.x << ", " << B.x << ", " << C.x;
            //std::cout << " [ay, by, cy] = " << A.y << ", " << B.y << ", " << C.y << std::endl;
        }

    }

    *arg_min = std::numeric_limits<int>::quiet_NaN(); // Just an initialization
    float min_iav = std::numeric_limits<float>::max();
    for ( unsigned int ii = 0; ii < iav.size() - 4; ++ii ) {
        float point_to_check = iav[ii+2];

        if ( point_to_check < min_iav && point_to_check == point_to_check ) {

            if ( iav[ii] == iav[ii] && iav[ii+1] == iav[ii+1] && iav[ii+3] == iav[ii+3]  && iav[ii+4] == iav[ii+4] ) {
                float left = iav[ii];
                float left_plus1 = iav[ii+1];
                float right_min1 = iav[ii+3];
                float right = iav[ii+4];

                // For corner detection? -> minimum in inscribed radius could be a corner
                // criterion for arg_min including robustness issues: 4 neighbouring points (2 on each side) must me progressively bigger
                // alternative: all must be larger than the point to check?
                bool check1 = left > left_plus1;
                bool check2 = left_plus1 > point_to_check;
                bool check3 = right_min1 > point_to_check;
                bool check4 = right > right_min1;

                if ( check1 && check2 && check3 && check4 ) {
                    min_iav = iav[ii];
                    *arg_min = ii;
                }
            }
        }
    }

    std::cout << "arg_min in inscribedRadius = " << *arg_min << std::endl;

// Determine mean and standard deviation of the inscribed radius
    unsigned int counter = 0;
    float sum = 0.0;
    for ( unsigned int i = 0; i < iav.size(); ++i ) {
        if ( iav[i] == iav[i] ) {
            sum += iav[i];
            ++counter;
        }
    }
    *mean = sum / counter;

    std::vector<float> diff ( counter );
    counter = 0;
    for ( unsigned int i = 0; i < iav.size(); ++i ) {
        if ( iav[i] == iav[i] ) {
            diff[counter] = iav[i] - *mean;
            ++counter;
        }
    }

    /* float sum = std::accumulate ( iav.begin(), iav.end(), 0.0 );
     *mean = sum / iav.size();

     std::vector<float> diff ( iav.size() );
     std::transform ( iav.begin(), iav.end(), diff.begin(),
                      std::bind2nd ( std::minus<float>(), *mean ) );*/

    float sq_sum = std::inner_product ( diff.begin(), diff.end(), diff.begin(), 0.0 );
    *std_dev = std::sqrt ( sq_sum / counter );

    std::cout << "[mean_iav, std_dev_iav] = [" << *mean << ", " << *std_dev << "]" << std::endl;

    if ( *arg_min > points.size() || *mean != *mean || *std_dev != *std_dev ) {
        std::cout << "WARNING: NUMBER OF POINTS TOO HIGH!" << std::endl;

        std::cout << "Points = " << std::endl;
        for ( unsigned int i = 0; i < points.size(); ++i ) {
            std::cout << points[i].x << ", " << points[i].y << ";";
        }
        std::cout << "\n";


        std::cout << "iav = " << std::endl;
        for ( unsigned int i = 0; i < iav.size(); ++i ) {
            std::cout << iav[i] << ";";
        }
        std::cout << "\n";

    } else {
        std::cout << "Points = " << std::endl;
        for ( unsigned int i = 0; i < points.size(); ++i ) {
            std::cout << points[i].x << ", " << points[i].y << ";";
        }
        std::cout << "\n";


        std::cout << "iav = " << std::endl;
        for ( unsigned int i = 0; i < iav.size(); ++i ) {
            std::cout << iav[i] << ";";
        }
        std::cout << "\n";
    }

    return iav;


}

//Fast Line, Arc/Circle and Leg Detection from Laser Scan Data in a Player Driver: http://miarn.sourceforge.net/pdf/a1738b.pdf

float fitCircle ( std::vector<geo::Vec2f>& points, ed::tracking::Circle* circle, geo::Pose3D& pose )
{

    // according to https://dtcenter.org/met/users/docs/write_ups/circle_fit.pdf
    float x_avg = 0.0, y_avg = 0.0;
    for ( unsigned int i = 0; i < points.size(); ++i ) {
//         std::cout << "Points = " << points[i].x << ", " << points[i].y << std::endl;
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

    float sum = 0.0;
    for ( unsigned int i = 0; i < points.size(); ++i ) {
        float error = fabs ( sqrt ( pow ( xc - points[i].x, 2.0 ) + pow ( yc - points[i].y, 2.0 ) ) - R ); // distance between a point and a circle;
        sum += error;
	/*if ( error > MAX_LINE_ERROR ) {
            return false;
        }
        */
    }

    circle->setValues ( xc, yc,  R );
    return sum/points.size();
}

void Circle::setValues ( float x, float y, float R )
{
    x_ = x;
    y_ = y;
    R_ = R;
}

void Circle::printValues ( )
{
    std::cout << "x_ = " << x_;
    std::cout << " y_ = " << y_;
    std::cout << " R_ = " << R_<< std::endl;
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

float fitRectangle ( std::vector<geo::Vec2f>& points, ed::tracking::Rectangle* rectangle, geo::Pose3D& pose , unsigned int* cornerIndex )
{
    /*
      for ( unsigned int i = 0; i < points.size(); ++i ) {
          std::cout << "Points = " << points[i].x << ", " << points[i].y << std::endl;
      }
    */
    // First determine the lines
    /*  float angle, angle1, angle2;
      //geo::Vec2f centroid, centroid1, centroid2;
      unsigned int cornerIndex, max_error_index1, max_error_index2;

      bool fitRectangle = false;
      bool fitSingleline = false;
      std::cout << "Start fitLine" << std::endl;

      bool splitData = findPossibleCorner ( points, cornerIndex );

      /* std::vector<float> errors = fitLine ( points, angle, centroid);//, max_error_index );
       float max_error = *std::max_element ( errors.begin(), errors.end() );
      */
    /*   std::cout << "splitData = " << splitData << " @ID " << cornerIndex << std::endl;
       //if ( max_error > MAX_LINE_ERROR ) {
       if ( splitData ) {
           std::cout << "Max error criterion not met, or (1) remove points and try to fit line again, or (2) split points and try to fit square" << std::endl;

           // check for nan first?
           unsigned int nPointsLow = cornerIndex;
           unsigned int nPointsHigh = points.size() - cornerIndex + 1; // +1 because the point with max error is considered as the corner point and belongs to both lines

           std::cout << "[nPointsLow nPointsHigh] = [" << nPointsLow << ", " << nPointsHigh << "]"  << std::endl;

           bool fitSingleline = false;
           bool pointsRemoved = false;
           if ( nPointsLow < MIN_POINTS_LINEFIT ) { // Part of section too smal -> remove it from the data which are analyzed and try to fit line again
               points.erase ( points.begin(), points.begin() + nPointsLow - 1 );
               pointsRemoved = true;
           }

           if ( nPointsHigh < MIN_POINTS_LINEFIT ) {
               points.erase ( points.end() - ( int )  nPointsHigh + 1,points.end() );
               pointsRemoved = true;
           }



           if ( pointsRemoved && points.size() >= MIN_POINTS_LINEFIT ) {
               fitSingleline = true;
           } else if ( pointsRemoved && points.size() < MIN_POINTS_LINEFIT ) {
               return false;
           } else {
               fitRectangle = true;
           }
       } else {
           fitSingleline = true;
       }

    */
    /*else if (nPointsLow != nPointsLow){

        std::vector<unsigned int> pointsToRemove;
        if ( errors[0]  > MAX_LINE_ERROR ) { // Error at end-points too large: remove it
            pointsToRemove.push_back ( 0 );
        }

        for ( unsigned int ii = 1; ii < errors.size() - 1; ++ii ) {
            bool check1 = errors[ii] > MAX_LINE_ERROR;
            bool check2 = errors[ii - 1] > MAX_LINE_ERROR ||  errors[ii + 1] > MAX_LINE_ERROR; // point itself and one of the neighbours have an error which is too big

            if ( check1 && check2 ) {
                pointsToRemove.push_back ( ii );
            }
        }

        if ( errors[errors.size()]  > MAX_LINE_ERROR ) {
            pointsToRemove.push_back ( errors.size() );
        }


        if ( pointsToRemove.size() > 0 ) {
            for ( unsigned int jj = 0; jj < pointsToRemove.size(); ++jj ) {
                points.erase ( points.begin() + pointsToRemove[jj] - jj );
            }

        } else {
            fit_rectangle = true;
        }
    }
    else {
       fit_rectangle = true;
    }

    std::cout << "Points after removal = " << std::endl;
    for ( unsigned int i = 0; i < points.size(); ++i ) {
        std::cout << points[i].x << ", " << points[i].y << ";";
    }
    std::cout << "\n";

    if ( redo_linefit ) {
        float mean_iav, std_dev_iav;
        std::vector<float> iav = ed::tracking::inscribedRadius ( points, &mean_iav, &std_dev_iav, &min_counter );
        std::vector<float> errors = fitLine ( points, angle, centroid);//, max_error_index );

        if ( *std::max_element(errors.begin(), errors.end() ) > MAX_LINE_ERROR ) {
            nPointsLow = min_counter;
            nPointsHigh = points.size() - min_counter + 1; // +1 because the point with max error is considered as the corner point and belongs to both lines
            if ( nPointsLow < MIN_POINTS_LINEFIT || nPointsHigh < MIN_POINTS_LINEFIT ) { // Minimal number of points required for splitting line
                return false;
            } else {
                fit_rectangle = true;
            }
        }
    }

    // check total length of vector
    // redo linefit and check if max error < max_line_error

    //||   ) { // If one of the segments is very small, remove it from the data to analyse
    }
    */




// std::cout << "Max error = " << max_error << "for " << points.size() << "Points" << std::endl;

    // if ( fitRectangle ) { // split data at max error and repeat for both left and right lines

    std::cout << "Try to fit rectangle in data" << std::endl;

    // For testing 
    //bool splitData = findPossibleCorner ( points, *cornerIndex );
    //if(!splitData){ return false; }
    
    std::cout << "Index = " <<  *cornerIndex << std::endl;
    
    std::vector<geo::Vec2f>::iterator it_split = points.begin() + *cornerIndex;
   // std::vector<geo::Vec2f> pointsLow ( points.begin(), it_split );
   // std::vector<geo::Vec2f> pointsHigh ( it_split, points.end() );

    //   std::cout << "Number of Points [low, high, total] = " << points_low.size() << ", " << points_hi.size()  << ", " << points.size() << ", " << std::endl;

    // std::cout << "Try to fit 2 lines" << std::endl;
    // std::cout << "fit Line 1 with " << points_low.size() << "points" << std::endl;

    Eigen::VectorXd beta_hat1 ( 2 ), beta_hat2 ( 2 );
    bool validFit1, validFit2;

    float mean_error1 = fitLine ( points, beta_hat1, points.begin(), it_split ); //, max_error_index1 );
      std::cout << "error 1 = " << mean_error1 << std::endl;

    //  std::cout << "fit Line 2 with"  << points_hi.size() << " points" << std::endl;
    float mean_error2 = fitLine ( points, beta_hat2, it_split, points.end() ); //, max_error_index2 );
      std::cout << "error 2 = " << mean_error2 << std::endl;


    //std::cout << "Errors determined" << std::endl;
   /* if ( !validFit1 || !validFit2 ) {
        std::cout << "WARNING: more line segments needed (is square assumption valid?)" << std::endl;
        //std::cout << " max_error1 = " << error1 << " max_error2 = " << error2 << std::endl;

        return false;
    }
    */

    //     std::cout << "Lines fitted" << std::endl;
	
	

    float reference_x = points[0].x;
    float reference_y = beta_hat1 ( 1 ) * reference_x + beta_hat1 ( 0 );

    std::cout << "Center determined" << std::endl;

    //determine width and height
    float x_end = points[*cornerIndex].x;
    float dx = reference_x - x_end;
    float dy = reference_y - ( beta_hat1 ( 1 ) * x_end + beta_hat1 ( 0 ) );
    float width = sqrt ( dx*dx+dy*dy );
    float theta = atan2 ( beta_hat1 ( 1 ), 1 ); // TODO: angle on points low alone?
    
    

//	std::cout << "theta before = " << theta << std::endl;
    //ed::tracking::wrapToInterval(&theta, 0.0, 2*M_PI);
    //std::cout << "theta after = " << theta << std::endl;

    //   std::cout << "Width and height determined" << std::endl;

    reference_x = points[*cornerIndex].x;
    reference_y = beta_hat2 ( 1 ) * reference_x + beta_hat2 ( 0 );
    
    x_end = points[points.size() - 1].x;
    dx = x_end - reference_x;
    dy = reference_y - ( beta_hat2 ( 1 ) * x_end + beta_hat2 ( 0 ) );
    float depth = sqrt ( dx*dx+dy*dy );
    std::cout << "dx and dy determined: dx = " << dx << " dy = " << dy << std::endl;

    rectangle->setValues ( reference_x, reference_y, width, depth, ARBITRARY_HEIGHT, theta );
    //     std::cout << "Values of rectangle set" << std::endl;
    //  }
    /*else { // Single line considered

        //     std::cout << "Single line considered" << std::endl;
        float center_x = points[points.size() - 1].x;//centroid.x;
        float center_y = points[points.size() - 1].y;//centroid.y;

        float dx = points[points.size()-1].x-points[0].x;
        float dy = points[points.size()-1].y-points[0].y;
        float width = sqrt ( dx*dx+dy*dy );
        float theta = atan2 ( dy, dx );


    // fit line, determine length using least-squares method to determine theta



        //std::cout << "theta before = " << theta << std::endl;
        //ed::tracking::wrapToInterval(&theta, 0.0, 2*M_PI);
    //std::cout << "theta after = " << theta << std::endl;

        //    std::cout << "Beginpoint = " << points[0].x << ", Endpoint = " << points[points.size()-1].x << std::endl;

        //      std::cout <<"dx, dy, width = " << dx  << ", " << dy  << ", " << width  << std::endl;

        //     std::cout << "Set values of rectangle (using a single line)" << std::endl;
        rectangle->setValues ( center_x, center_y, width, ARBITRARY_DEPTH, ARBITRARY_HEIGHT, theta );

    }
    */

    std::cout << "End of fitRectangle function" << std::endl;

    unsigned int low_size = *cornerIndex;
    unsigned int high_size = points.size() - *cornerIndex + 1;
    return (mean_error1*low_size+mean_error2*high_size )/(low_size + high_size); // weighted average of error

}

bool findPossibleCorner ( std::vector<geo::Vec2f>& points, unsigned int &ID )
{
    geo::Vec2f startPoint = points[0];
    geo::Vec2f endPoint = points[points.size() - 1];

    float a = endPoint.y-startPoint.y;
    float b = endPoint.x-startPoint.x;
    float c = endPoint.x*startPoint.y-endPoint.y*startPoint.x;

    float length = sqrt ( pow ( a,2.0 ) + pow ( b,2.0 ) );

    float maxDistance = 0.0;

//std::cout <<"[a, b, c,  length ] = [" << a << ", " << b << ", " << c << ", " << length << "]" << std::endl;

    ID = std::numeric_limits<unsigned int>::quiet_NaN();
    for ( unsigned int ii = 1; ii < points.size() - 1; ++ii ) {
        float distance = fabs ( a* ( points[ii].x )-b* ( points[ii].y ) +c ) /length;

        // std::cout << "Distance = " << distance << " @ ii = " << ii << " x = " << points[ii].x << " y = " << points[ii].y  << "intermediate = " << a*points[ii].x << ", " << b*(points[ii].y) << std::endl;
        if ( distance > maxDistance ) {
            maxDistance = distance;
            ID = ii;
        }
    }

//std::cout << "maxDistance = " << maxDistance << std::endl;

    if ( maxDistance >  MIN_DISTANCE_CORNER_DETECTION ) {
        return true;
    } else {
        return false;
    }
    

}

float fitLine ( std::vector<geo::Vec2f>& points, Eigen::VectorXd& beta_hat, std::vector<geo::Vec2f>::iterator it_start, std::vector<geo::Vec2f>::iterator it_end )  //, unsigned int& index ) // TODO: centroid not used anymore, can be removed
{
    std::cout << "FitLine function started" << std::endl;
    // Least squares method: http://home.isr.uc.pt/~cpremebida/files_cp/Segmentation%20and%20Geometric%20Primitives%20Extraction%20from%202D%20Laser%20Range%20Data%20for%20Mobile%20Robot%20Applications.pdf
    
    unsigned int size = std::distance(it_start, it_end);
    
    std::cout << "nPoints in fitline = " << size << std::endl;
    
    Eigen::MatrixXd m ( size, 2 );
    Eigen::VectorXd y ( size );

    unsigned int counter = 0;
    unsigned int start = std::distance(points.begin(), it_start);
    for ( std::vector<geo::Vec2f>::iterator it = it_start; it != it_end; ++it ) {
        m ( counter, 0 ) = ( double ) 1.0;
        m ( counter, 1 ) = ( double ) points[start + counter].x;
        y ( counter ) = ( double ) points[start + counter].y;
	counter++;
    }
    
  //  std::cout << "m = " << m << std::endl;
  //  std::cout << "y = " << y << std::endl;
    
    Eigen::MatrixXd mt (size, 2 );
    mt = m.transpose();

    //Eigen::VectorXd beta_hat ( 2 );
    beta_hat = ( mt*m ).inverse() * mt * y;

    // angle = atan2 ( beta_hat ( 1 ), 1 );

    float error, sum = 0.0;
    
    //std::vector<EntityUpdate>::const_iterator it = clusters.begin(); it != clusters.end(); ++it 
    
    counter = 0;
    for ( std::vector<geo::Vec2f>::iterator it = it_start; it != it_end; ++it ) {
        // Distance of each point to line
        error = fabs ( -beta_hat ( 1 ) * points[start + counter].x+points[start + counter].y - beta_hat ( 0 ) ) /sqrt ( beta_hat ( 1 ) *beta_hat ( 1 ) + 1 ); // distance of a point to a line
	sum += error;
	counter ++;
       /* if ( error > MAX_LINE_ERROR ) {
            return false;
        }
        */
    }
    
    std::cout << "counter = " << counter << std::endl;

    return sum/counter;


    /*
      float x_sum = 0.0, y_sum = 0.0;
      for ( unsigned int i = 0; i < points.size(); ++i ) {
          //   std::cout << "Points = " << points[i].x << ", " << points[i].y << std::endl;
          x_sum += points[i].x;
          y_sum += points[i].y;
      }

      centroid.x  = x_sum / points.size();
      centroid.y  = y_sum / points.size();

      std::cout << "[x_centroid, y_centroid] = " << centroid.x << ", " << centroid.y << std::endl;

      std::vector<geo::Vec2f> points_corrected = points;
      float sum_xy = 0.0, sum_squared_diff = 0.0;
      for ( unsigned int i = 0; i < points.size(); ++i ) {
          //std::cout << "Points = " << points[i].x << ", " << points[i].y << std::endl;
          points_corrected[i].x -= centroid.x;
          points_corrected[i].y -= centroid.y;

          sum_xy += points_corrected[i].x * points_corrected[i].y;
          sum_squared_diff += points_corrected[i].x * points_corrected[i].x - points_corrected[i].y * points_corrected[i].y;
      }

      //std::cout << "fitLine 1" << std::endl;

      float A = sum_squared_diff/sum_xy;
      float tan_q1 = 0.5* ( -A - sqrt ( ( A*A+4 ) ) );
      float tan_q2 = 0.5* ( -A + sqrt ( ( A*A+4 ) ) );

      float angle1 = atan ( tan_q1 );
      float angle2 = atan ( tan_q2 );

      float error1 = 0.0, error2 = 0.0, max_error1 = 0.0, max_error2 = 0.0, max_error, error; // max error of line with best fit
      unsigned int error_index1, error_index2;
      // 2 lines will be possible from the solution: one with maximum error and one with minimal error. We are looking for the one with minimal error

      for ( unsigned int i = 0; i < points.size(); ++i ) {
          // Distance of each point to line
          error = fabs ( angle1*points_corrected[i].x+points_corrected[i].y ) /sqrt ( angle1*angle1+1 ); // distance of a point to a line
          error1 += error;

          if ( error > max_error1 ) {
              max_error1 = error;
              error_index1 = i;
          }

          error = fabs ( angle2*points_corrected[i].x+points_corrected[i].y ) /sqrt ( angle2*angle2+1 );
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

      return max_error; // Maximum error of the line with the best fit
      */


}

void Rectangle::setValues ( float x, float y, float w, float d, float h, float theta )
{
    x_ = x;
    y_ = y;
    w_ = w;
    d_ = d;
    h_ = h;
    theta_ = theta;
}

void Rectangle::printValues ( )
{
    std::cout << "x_ = " << x_;
    std::cout << " y_ = " << y_;
    std::cout << " w_ = " << w_;
    std::cout << " d_ = " << d_;
    std::cout << " h_ = " << h_;
    std::cout << " theta_ = " << theta_ << std::endl;
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
    marker.pose.position.x = x_ - 0.5* ( w_*cos ( theta_ ) - d_*sin ( theta_ ) );
    marker.pose.position.y = y_ - 0.5* ( w_*sin ( theta_ ) + d_*cos ( theta_ ) );
    marker.pose.position.z = 0.5; // TODO: on height of laser

    //std::cout << "MArker: x_, y_, R_" << x_ << ", " << y_ << ", " << R_ << std::endl;

    //marker.pose.orientation = geometry_msgs::geometry_msgs

//quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw);


    /*marker.pose.orientation.x = cos(theta_/2);
    marker.pose.orientation.y = 0;//sin(theta_/2);
    marker.pose.orientation.z = sin ( theta_/2 );
    marker.pose.orientation.w = 0;//sin ( theta_/2 );
    */

    // marker.pose.orientation = createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( 0.0, 0.0, theta_ );
    marker.scale.x = w_;
    marker.scale.y = d_;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.lifetime = ros::Duration ( TIMEOUT_TIME );
}

void wrapToInterval ( float* alpha, float lowerBound, float upperBound )
{
    float delta = upperBound - lowerBound;

    if ( *alpha < lowerBound ) {
        while ( *alpha < lowerBound ) {
            *alpha += delta;

            std::cout << "alpha = " << alpha  << std::endl;
        }
    } else if ( *alpha >= lowerBound ) {
        while ( *alpha >= lowerBound ) {
            *alpha -= delta;
            std::cout << "alpha = " << alpha  << std::endl;
        }
    }

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
