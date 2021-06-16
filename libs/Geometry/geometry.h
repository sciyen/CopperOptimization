#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Exact_circular_kernel_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/intersections.h>

#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_circle_segment_traits_2.h>
#include <CGAL/Surface_sweep_2_algorithms.h>

#include <iostream>
#include <string>
#include <list>
#include <vector>

#include <cmath>
#include <opencv2/opencv.hpp>

// Line
//typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Line_2 Line_2;
typedef Kernel::Segment_2 Segment_2;
typedef Kernel::Circle_2 Circle_2;
typedef Kernel::Vector_2 Vector_2;

// Circular Arc
// http://cgal-discuss.949826.n4.nabble.com/Convert-points-and-circles-from-2D-linear-kernel-to-2D-circular-kernel-td4663254.html
// Bring in the 2D arrangement traits
typedef CGAL::Arr_circle_segment_traits_2<Kernel> Circle_Traits_2;
// We need the circular entities from the 2D trait
typedef Circle_Traits_2::Point_2 Arrangement_Point_2;
typedef Circle_Traits_2::Curve_2 Arrangement_Curve_2;

typedef std::list<Arrangement_Curve_2> CurveList;
typedef CurveList::iterator CurveListIter;

typedef CGAL::Exact_circular_kernel_2 Circular_k;
typedef CGAL::Circle_2<Circular_k> Circular_Circle_2;
typedef CGAL::Point_2<Circular_k> Circular_Point_2;
typedef CGAL::Line_2<Circular_k> Circular_Line_2;
typedef CGAL::Circular_arc_point_2<Circular_k> Circular_Arc_Point_2;
typedef CGAL::Circular_arc_2<Circular_k> Circular_arc_2;

// Bounding box
typedef CGAL::Bbox_2 Bbox_2;

// Collision
typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_2 Exact_Point_2;
typedef K::Segment_2 Exact_Segment_2;
typedef K::Line_2 Exact_Line_2;

typedef K::Intersect_2 Intersect_2;
typedef K::Iso_rectangle_2 Iso_rectangle_2;

// Translation
typedef CGAL::Aff_transformation_2<Kernel> Transformation;

enum NodeType { UNKNOWN, DRILL, SMD, PTH };
enum GeoType { GEO_UNKNOWN, GEO_LINE, GEO_CIRCLE, GEO_ARC };

/* This struct record the parameters that are required to
 * translate from pcb coordinate to window coordinate. */
typedef struct {
    double s_xmin;  // source x min
    double s_ymin;  // source y min
    double s_w;     // source width
    double s_h;     // source height
    int t_w;        // target width
    int t_h;        // target height
    double scale;
} DrawConfig;

/* Virtual base class of simple 2D geometries */
class Geometry
{
protected:
    int parse(std::string s, double *params);

    static void translate(double &ox,
                          double &oy,
                          const Point_2 p,
                          const DrawConfig &dc);

public:
    GeoType geo_type;
    CGAL::Bbox_2 bbox;
    Geometry();
    virtual void add_feature(const std::string &s) = 0;

    virtual Bbox_2 get_bbox() = 0;

    /* Expand the geometry with the given distance and center. */
    virtual void dilate(Point_2 center, double dis) = 0;

    virtual Arrangement_Curve_2 gen_curve() = 0;
    virtual void check_collision(Geometry &g,
                                 std::vector<Point_2> &res) = 0;

    virtual void move(Vector_2 v) = 0;

    /* Every children class should implement this function,
     * which draws its own geometry with opencv functions. */
    virtual void draw(cv::Mat img,
                      const DrawConfig &dc,
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
    virtual void dilate(Point_2 center, double dis);
    virtual void check_collision(Geometry &g, std::vector<Point_2> &res);
    virtual Arrangement_Curve_2 gen_curve();
    virtual void move(Vector_2 v);
    virtual void draw(cv::Mat img,
                      const DrawConfig &dc,
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

    bool dir;  // true for cw, false for ccw
    GeometryArc(const std::string &s);
    virtual void add_feature(const std::string &s);
    virtual Bbox_2 get_bbox();
    virtual void dilate(Point_2 center, double dis);
    virtual void check_collision(Geometry &g, std::vector<Point_2> &res);
    virtual Arrangement_Curve_2 gen_curve();
    virtual void move(Vector_2 v);
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
    bool is_collision;
    CGAL::Bbox_2 bbox;
    Point_2 center;
    // std::string node_type;
    NodeType node_type;

    std::vector<std::shared_ptr<Geometry>> geometries;

    /* Constructor for Node
     * @Params:
     *      node_type: "drill", "pth", "smd"
     *      geos_str:  geometries descriptor.
     */
    Node(const std::string &_node_type,
         const std::vector<std::string> &geos_str);
    ~Node();
    Bbox_2 get_bbox();
    void dilate(double dis);
    void print_bbox();
    void move(Vector_2 v);

    /* Check if a collision occurred with the given node. */
    bool check_collision(const Node &n);

    void draw(cv::Mat img, const DrawConfig &dc);
};

#endif