#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <iostream>
#include <string>
#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_circular_kernel_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

#include <cmath>
#include <opencv2/opencv.hpp>

// Line
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Segment_2 Segment_2;
typedef Kernel::Circle_2 Circle_2;
typedef Kernel::Vector_2 Vector_2;

// Circular Arc
typedef CGAL::Exact_circular_kernel_2 Circular_k;
typedef CGAL::Point_2<Circular_k> Circular_Point_2;
typedef CGAL::Circular_arc_point_2<Circular_k> Circular_Arc_Point_2;
typedef CGAL::Circular_arc_2<Circular_k> Circular_arc_2;

// Bounding box
typedef CGAL::Bbox_2 Bbox_2;

// Collision
typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 Exact_Point_2;
typedef K::Intersect_2 Intersect_2;
typedef K::Iso_rectangle_2 Iso_rectangle_2;

/* This struct record the parameters that are required to 
 * translate from pcb coordinate to window coordinate. */
typedef struct
{
    double s_xmin; // source x min
    double s_ymin; // source y min
    double s_w;    // source width
    double s_h;    // source height
    int t_w;       // target width
    int t_h;       // target height
    double scale;
} DrawConfig;

/* Virtual base class of simple 2D geometries */
class Geometry
{
protected:
    int parse(std::string s, double *params);

    static void translate(double &ox, double &oy, const Point_2 p, const DrawConfig &dc);

public:
    CGAL::Bbox_2 bbox;
    Geometry();
    virtual void add_feature(const std::string &s) = 0;

    virtual Bbox_2 get_bbox() = 0;

    /* Every children class should implement this function, 
     * which draws its own geometry with opencv functions. */
    virtual void draw(cv::Mat img, const DrawConfig &dc,
                      const cv::Scalar color = cv::Scalar(255, 0, 0),
                      const int thickness = 2,
                      const int lineType = cv::LINE_8) = 0;
};

class GeometryLine : public Geometry
{
private:
    Segment_2 seg;

public:
    GeometryLine(const std::string &s);
    virtual void add_feature(const std::string &s);
    virtual Bbox_2 get_bbox();
    virtual void draw(cv::Mat img, const DrawConfig &dc,
                      const cv::Scalar color = cv::Scalar(255, 0, 0),
                      const int thickness = 2,
                      const int lineType = cv::LINE_8);
};

class GeometryArc : public Geometry
{
private:
    bool is_circle;

public:
    Circle_2 circle;
    Circular_arc_2 arc;

    bool dir; // true for cw, false for ccw
    GeometryArc(const std::string &s);
    virtual void add_feature(const std::string &s);
    virtual Bbox_2 get_bbox();
    virtual void draw(cv::Mat img,
                      const DrawConfig &dc,
                      const cv::Scalar color = cv::Scalar(255, 0, 0),
                      const int thickness = 2,
                      const int lineType = cv::LINE_8);
};

class Node
{
private:
public:
    CGAL::Bbox_2 bbox;
    Point_2 center;
    std::string node_type;

    std::vector<std::shared_ptr<Geometry>> geometries;

    /* Constructor for Node
     * @Params:
     *      node_type: "drill", "pth", "smd"
     *      geos_str:  geometries descriptor.
     */
    Node(const std::string &node_type, const std::vector<std::string> &geos_str);
    ~Node();
    Bbox_2 get_bbox();
    void print_bbox();

    /* Check if a collision occurred with the given node. */
    bool check_collision(const Node &n);

    void draw(cv::Mat img,
              const DrawConfig &dc);
};

#endif