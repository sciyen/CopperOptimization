#include "geometry.h"

Geometry::Geometry() {}

Bbox_2 Geometry::get_bbox()
{
    return CGAL::Bbox_2();
}

int Geometry::parse(std::string s, double *params)
{
    int i, pre = 0;
    size_t p = 0;
    for (i = 0; (p = s.find(',', pre)) != std::string::npos; i++) {
        params[i] = std::stof(s.substr(pre, p - pre));
        pre = p + 1;
    }
    std::string last = s.substr(pre);
    try {
        params[i] = std::stof(last);
    } catch (const std::invalid_argument &ia) {
        // compare CW: 1, CCW: -5140
        params[i] = (last.compare("CW") > 0) ? 1 : -1;
    }
    return i;
}

void Geometry::translate(double &ox,
                         double &oy,
                         const Point_2 p,
                         const DrawConfig &dc)
{
    ox = (p.x() - dc.s_xmin) * dc.t_w / dc.s_w;
    oy = (p.y() - dc.s_ymin) * dc.t_h / dc.s_h;
}

GeometryLine::GeometryLine(const std::string &geo) : Geometry()
{
    add_feature(geo);
    bbox = get_bbox();
};

void GeometryLine::add_feature(const std::string &s)
{
    double x1, y1, x2, y2;
    double buf[4];
    parse(s, buf);
    x1 = buf[0];
    y1 = buf[1];
    x2 = buf[2];
    y2 = buf[3];
    std::cout << "line: " << x1 << ',' << y1 << ',' << x2 << ',' << y2
              << std::endl;

    seg = Segment_2(Point_2(x1, y1), Point_2(x2, y2));
}

Bbox_2 GeometryLine::get_bbox()
{
    std::cout << "arc" << std::endl;
    return seg.bbox();
}

void GeometryLine::draw(cv::Mat img,
                        const DrawConfig &dc,
                        const cv::Scalar color,
                        const int thickness,
                        const int lineType)
{
    double x1, y1, x2, y2;
    this->translate(x1, y1, seg.source(), dc);
    this->translate(x2, y2, seg.target(), dc);

    cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), color,
             thickness, lineType);
}

GeometryArc::GeometryArc(const std::string &geo) : Geometry()
{
    is_circle = false;
    add_feature(geo);
};

void GeometryArc::add_feature(const std::string &s)
{
    double ax1, ay1, ax2, ay2, cx, cy, r;
    double buf[8];
    int num = parse(s, buf);
    int offset = 0;
    ax1 = buf[0];
    ay1 = buf[1];
    ax2 = buf[2];
    ay2 = buf[3];
    cx = buf[4];
    cy = buf[5];
    if (num == 7) {
        is_circle = true;
        r = buf[6];
        offset = 1;
    }
    dir = (buf[offset + 6] > 0) ? true : false;
    std::cout << "Arc: " << ax1 << ',' << ay1 << ',' << ax2 << ',' << ay2 << ','
              << cx << ',' << cy << ',' << (dir ? "CW" : "CCW") << std::endl;

    if (is_circle)
        // center, radius, orientation=COUNTERCLOCKWISE
        circle = Circle_2(Point_2(cx, cy), r);
    else {
        // DOTO: orientation
        // center, arc_point, arc_point
        if (dir)
            arc = Circular_arc_2(
                Circular_Point_2(cx, cy),
                Circular_Arc_Point_2(Circular_Point_2(ax2, ay2)),
                Circular_Arc_Point_2(Circular_Point_2(ax1, ay1)));
        else
            arc = Circular_arc_2(
                Circular_Point_2(cx, cy),
                Circular_Arc_Point_2(Circular_Point_2(ax1, ay1)),
                Circular_Arc_Point_2(Circular_Point_2(ax2, ay2)));
    }
}

Bbox_2 GeometryArc::get_bbox()
{
    std::cout << "arc" << std::endl;

    if (is_circle)
        return circle.bbox();
    else
        return arc.bbox();
}

void GeometryArc::draw(cv::Mat img,
                       const DrawConfig &dc,
                       const cv::Scalar color,
                       const int thickness,
                       const int lineType)
{
    double cx, cy;

    if (is_circle) {
        this->translate(cx, cy, circle.center(), dc);
        double r = (CGAL::to_double(circle.squared_radius())) * dc.scale;
        cv::circle(img, cv::Point(cx, cy), r, color, thickness,
                   lineType);
    } else {
        double cx = CGAL::to_double(arc.center().x());
        double cy = CGAL::to_double(arc.center().y());
        double sx = CGAL::to_double(arc.source().x());
        double sy = CGAL::to_double(arc.source().y());
        double ex = CGAL::to_double(arc.target().x());
        double ey = CGAL::to_double(arc.target().y());
        double start_angle = atan2(sy - cy, sx - cx) * 180 / M_PI;
        double end_angle = atan2(ey - cy, ex - cx) * 180 / M_PI;
        Point_2 center = Point_2(cx, cy);

        // double r = (CGAL::to_double(arc.squared_radius())) * dc.scale;
        double r = sqrt(pow(sx - cx, 2) + pow(sy - cy, 2)) * dc.scale;
        std::cout << "axis" << sx << ',' << sy << ',' << cx << ',' << cy << ','
                  << r << std::endl;
        std::cout << "angle" << start_angle << ',' << end_angle << ',' << r
                  << std::endl;

        this->translate(cx, cy, center, dc);
        cv::ellipse(img, cv::Point(cx, cy), cv::Size(r, r), 0, start_angle,
                    end_angle, color, thickness, lineType);
    }
}

Node::Node(const std::string &_node_type,
           const std::vector<std::string> &_geos_str)
{
    node_type = _node_type;
    bbox = CGAL::Bbox_2();
    for (auto geo : _geos_str) {
        int start = geo.find(',');
        auto geo_type = geo.substr(0, start);

        // initialize according the type of geometry
        /*Geometry *g;
        if (geo_type == "line")
            g = new GeometryLine(geo.substr(start + 1));
        else if (geo_type == "arc")
            g = new GeometryArc(geo.substr(start + 1));
        geometries.push_back(g);*/

        if (geo_type == "line")
            geometries.push_back(
                std::make_shared<GeometryLine>(geo.substr(start + 1)));
        else if (geo_type == "arc")
            geometries.push_back(
                std::make_shared<GeometryArc>(geo.substr(start + 1)));
    }
}

Bbox_2 Node::get_bbox()
{
    bbox = CGAL::Bbox_2();
    std::cout << "type: " << node_type << std::endl;
    for (auto g : geometries) {
        // Bbox_2 gb = g->get_bbox();
        // std::cout << gb.xmin() << std::endl;
        bbox += g->get_bbox();
        // bbox += g->bbox;
    }
    center = Point_2((bbox.xmin() + bbox.xmax()) / 2,
                     (bbox.ymin() + bbox.ymax()) / 2);
    return bbox;
}

void Node::print_bbox()
{
    std::cout << bbox.xmin() << ',' << bbox.ymin() << ',' << bbox.xmax() << ','
              << bbox.ymax() << std::endl;
}

bool Node::check_collision(const Node &n)
{
    Iso_rectangle_2 rec0(Exact_Point_2(bbox.xmin(), bbox.ymin()),
                         Exact_Point_2(bbox.xmax(), bbox.ymax()));
    Iso_rectangle_2 rec1(Exact_Point_2(n.bbox.xmin(), n.bbox.ymin()),
                         Exact_Point_2(n.bbox.xmax(), n.bbox.ymax()));

    CGAL::cpp11::result_of<Intersect_2(Iso_rectangle_2, Iso_rectangle_2)>::type
        result = CGAL::intersection(rec0, rec1);
    if (result) {
        const Iso_rectangle_2 *s = boost::get<Iso_rectangle_2>(&*result);
        return true;
    }
    return false;
}

void Node::draw(cv::Mat img, const DrawConfig &dc)
{
    for (auto g : geometries) {
        cv::Scalar color;
        if (node_type == "drill")
            color = cv::Scalar(0, 255, 0);
        else
            color = cv::Scalar(255, 0, 0);

        g->draw(img, dc, color);
    }
}

Node::~Node()
{
    /*for (auto it = geometries.begin(); it < geometries.end(); it++)
        delete *it;*/
    // std::cout << "desc called" << std::endl;
}