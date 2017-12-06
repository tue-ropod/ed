#include "ed/convex_hull_calc.h"

#include <opencv2/imgproc/imgproc.hpp>

namespace ed
{

namespace tracking{

  void fitCircle(const std::vector<geo::Vec2f>& points, ed::tracking::Circle* cirlce, float z_min, float z_max, geo::Pose3D& pose)
  {
    
    // according to https://dtcenter.org/met/users/docs/write_ups/circle_fit.pdf
    float x_avg = 0.0, y_avg = 0.0;
    for(unsigned int i = 0; i < points.size(); ++i)
    {
      std::cout << "Points = " << points[i].x << ", " << points[i].y << std::endl;
      x_avg += points[i].x;
      y_avg += points[i].y;
    }
    
    x_avg /= points.size();
    y_avg /= points.size();
    
    std::vector<float> ui(points.size()), vi(points.size());
    float Suu = 0.0, Suv = 0.0, Suuu = 0.0, Suvv = 0.0, Svv = 0.0, Svvv = 0.0, Svuu = 0.0;
    for(unsigned int i = 0; i < points.size(); ++i)
    {
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
    float c = 0.5*(Suuu+Suvv);
    float d = Suv;
    float e = Svv;
    float f = 0.5*(Svvv+Svuu);
    
    float vc = (f - c*d/a)/(e-b*d/a);
    float uc = (c-vc*b)/a;

    float xc = uc+x_avg;
    float yc = vc+y_avg;

    float alpha = uc*uc+vc*vc+(Suu+Svv)/points.size();
    float R = std::sqrt(alpha);
std::cout << "fitter: x_, y_, R_" << xc << ", " << yc << ", " << R << std::endl;
    cirlce->setValues( xc, yc,  R);    
  }
  
  void Circle::setValues(float a, float b, float c){
  x_ = a;
  y_ = b;
  R_ = c;
}

void Circle::setMarker ( visualization_msgs::Marker& marker ){
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
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
  
  marker.lifetime = ros::Duration(1.0);
}
  
}

namespace convex_hull
{
// ----------------------------------------------------------------------------------------------------

void create(const std::vector<geo::Vec2f>& points, float z_min, float z_max, ConvexHull& chull, geo::Pose3D& pose)
{
    cv::Mat_<cv::Vec2f> points_2d(1, points.size());
    for(unsigned int i = 0; i < points.size(); ++i)
        points_2d.at<cv::Vec2f>(i) = cv::Vec2f(points[i].x, points[i].y);

    pose = geo::Pose3D::identity();

    pose.t.z = (z_min + z_max) / 2;

    std::vector<int> chull_indices;
    cv::convexHull(points_2d, chull_indices);

    chull.z_min = z_min - pose.t.z;
    chull.z_max = z_max - pose.t.z;

    geo::Vec2f xy_min(1e9, 1e9);
    geo::Vec2f xy_max(-1e9, -1e9);

    chull.points.clear();
    for(unsigned int i = 0; i < chull_indices.size(); ++i)
    {
        const cv::Vec2f& p_cv = points_2d.at<cv::Vec2f>(chull_indices[i]);
        geo::Vec2f p(p_cv[0], p_cv[1]);

        chull.points.push_back(p);

        xy_min.x = std::min(xy_min.x, p.x);
        xy_min.y = std::min(xy_min.y, p.y);

        xy_max.x = std::max(xy_max.x, p.x);
        xy_max.y = std::max(xy_max.y, p.y);
    }

    // Average segment position
    pose.t.x = (xy_min.x + xy_max.x) / 2;
    pose.t.y = (xy_min.y + xy_max.y) / 2;

    // Move all points to the pose frame
    for(unsigned int i = 0; i < chull.points.size(); ++i)
    {
        geo::Vec2f& p = chull.points[i];
        p.x -= pose.t.x;
        p.y -= pose.t.y;
    }

    // Calculate normals and edges
    convex_hull::calculateEdgesAndNormals(chull);

    // Calculate area
    calculateArea(chull);
}

// ----------------------------------------------------------------------------------------------------

void createAbsolute(const std::vector<geo::Vec2f>& points, float z_min, float z_max, ConvexHull& c)
{
    cv::Mat_<cv::Vec2f> points_2d(1, points.size());
    for(unsigned int i = 0; i < points.size(); ++i)
        points_2d.at<cv::Vec2f>(i) = cv::Vec2f(points[i].x, points[i].y);

    c.z_min = z_min;
    c.z_max = z_max;

    std::vector<int> chull_indices;
    cv::convexHull(points_2d, chull_indices);

    c.points.resize(chull_indices.size());
    for(unsigned int i = 0; i < chull_indices.size(); ++i)
    {
        const cv::Vec2f& p_cv = points_2d.at<cv::Vec2f>(chull_indices[i]);
        c.points[i] = geo::Vec2f(p_cv[0], p_cv[1]);
    }

    // Calculate normals and edges
    convex_hull::calculateEdgesAndNormals(c);

    // Calculate area
    calculateArea(c);
}

// ----------------------------------------------------------------------------------------------------

void calculateEdgesAndNormals(ConvexHull& c)
{
    c.edges.resize(c.points.size());
    c.normals.resize(c.points.size());

    for(unsigned int i = 0; i < c.points.size(); ++i)
    {
        unsigned int j = (i + 1) % c.points.size();

        const geo::Vec2f& p1 = c.points[i];
        const geo::Vec2f& p2 = c.points[j];

        // Calculate edge
        geo::Vec2f e = p2 - p1;
        c.edges[i] = e;

        // Calculate normal
        c.normals[i] = geo::Vec2f(e.y, -e.x).normalized();
    }
}

// ----------------------------------------------------------------------------------------------------

bool collide(const ConvexHull& c1, const geo::Vector3& pos1,
             const ConvexHull& c2, const geo::Vector3& pos2,
             float xy_padding, float z_padding)
{
    if (c1.points.size() < 3 || c2.points.size() < 3)
        return false;

    float z_diff = pos2.z - pos1.z;

    if (c1.z_max < (c2.z_min + z_diff - 2 * z_padding) || c2.z_max < (c1.z_min - z_diff - 2 * z_padding))
        return false;

    geo::Vec2f pos_diff(pos2.x - pos1.x, pos2.y - pos1.y);

    for(unsigned int i = 0; i < c1.points.size(); ++i)
    {
        const geo::Vec2f& p1 = c1.points[i];
        const geo::Vec2f& n = c1.normals[i];

        // Calculate min and max projection of c1
        float min1 = n.dot(c1.points[0] - p1);
        float max1 = min1;
        for(unsigned int k = 1; k < c1.points.size(); ++k)
        {
            // Calculate projection
            float p = n.dot(c1.points[k] - p1);
            min1 = std::min(min1, p);
            max1 = std::max(max1, p);
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
        for(unsigned int k = 0; k < c2.points.size(); ++k)
        {
            // Calculate projection on p1's normal
            float p = n.dot(c2.points[k] - p1_c2);

            below = below || (p < max1);
            above = above || (p > min1);

            if (below && above)
            {
                // There is overlap with c1's bound, so we may have a collision
                no_collision = false;
                break;
            }
        }

        if (no_collision)
            // definitely no collision
            return false;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

void calculateArea(ConvexHull& c)
{
    c.area = 0;
    for(unsigned int i = 0; i < c.points.size(); ++i)
    {
        unsigned int j = (i + 1) % c.points.size();

        const geo::Vec2f& p1 = c.points[i];
        const geo::Vec2f& p2 = c.points[j];

        c.area += 0.5 * (p1.x * p2.y - p2.x * p1.y);
    }
}

// ----------------------------------------------------------------------------------------------------

}

}
