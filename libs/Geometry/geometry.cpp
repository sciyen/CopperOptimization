#include "geometry.h"

Geometry::Geometry()
{
    geo_type = GEO_UNKNOWN;
}

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
    ox = CGAL::to_double(p.x() - dc.s_xmin) * dc.t_w / dc.s_w;
    oy = CGAL::to_double(p.y() - dc.s_ymin) * dc.t_h / dc.s_h;
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
    geo_type = GEO_LINE;
    curve = Arrangement_Curve_2(seg);
}

Bbox_2 GeometryLine::get_bbox()
{
    return seg.bbox();
}

void GeometryLine::dilate(Point_2 center, double dis)
{
    Vector_2 v =
        Vector_2(seg.target(), seg.source()).perpendicular(CGAL::LEFT_TURN);
    Vector_2 w = Vector_2(center, seg.source());
    double inner = CGAL::to_double(v.hx() * w.hx() + v.hy() * w.hy());
    double len =
        sqrt(pow(CGAL::to_double(v.x()), 2) + pow(CGAL::to_double(v.y()), 2));
    if (inner < 0)
        len *= -1;
    v = v * dis / len;
    Transformation translate(CGAL::TRANSLATION, v);
    seg = Segment_2(translate(seg.source()), translate(seg.target()));
    curve = Arrangement_Curve_2(seg);
}

void GeometryLine::check_collision(Geometry &g, std::vector<Point_2> &res)
{
    if (g.geo_type == GEO_LINE) {
        // two line
        Point_2 test_pt;
        Segment_2 test_seg;
        CGAL::Object obj =
            CGAL::intersection(seg, dynamic_cast<GeometryLine *>(&g)->seg);
        if (CGAL::assign(test_pt, obj)) {
            // the result is a point
            res.push_back(test_pt);
        } else if (CGAL::assign(test_seg, obj)) {
            // the result is a segment
            res.push_back(test_seg.source());
            res.push_back(test_seg.target());
        }
    } else {
        CurveList curves;
        curves.push_back(Arrangement_Curve_2(seg));
        Arrangement_Curve_2 c;
        if (g.geo_type == GEO_CIRCLE) {
            c = Arrangement_Curve_2(dynamic_cast<GeometryArc *>(&g)->circle);
        } else if (g.geo_type == GEO_ARC) {
            Circular_arc_2 arc = dynamic_cast<GeometryArc *>(&g)->arc;
            c = Arrangement_Curve_2(
                    Circle_2(Point_2(CGAL::to_double(arc.center().x()), 
                                     CGAL::to_double(arc.center().y())), 
                    CGAL::to_double(arc.squared_radius())),
                Arrangement_Point_2(CGAL::to_double(arc.source().x()), CGAL::to_double(arc.source().y())),
                Arrangement_Point_2(CGAL::to_double(arc.target().x()), CGAL::to_double(arc.target().y())));

            /*c = Arrangement_Curve_2(
                Circle_2(Point_2(0, 0), 5.0),
                Arrangement_Point_2(5.0, 0),
                Arrangement_Point_2(-5.0, 0));*/
        }
        curves.push_back(c);
        // Compute all intersection points.

        std::list<Arrangement_Point_2> pts;
        CGAL::compute_intersection_points(curves.begin(), 
                                    curves.end(),
                                    std::back_inserter(pts),
                                    false);
    }
}

Arrangement_Curve_2 GeometryLine::gen_curve(){
    return curve;
    return Arrangement_Curve_2(seg);
}

void GeometryLine::move(Vector_2 v) {}

void GeometryLine::draw(cv::Mat img,
                        const DrawConfig &dc,
                        const cv::Scalar color,
                        const int thickness,
                        const int lineType)
{
    double x1, y1, x2, y2;
    this->translate(x1, y1, seg.source(), dc);
    this->translate(x2, y2, seg.target(), dc);

    cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), color, thickness,
             lineType);
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
        r = pow(buf[6], 2);
        offset = 1;
    } else if ((ax1 == ax2) && (ay1 == ay2)) {
        is_circle = true;
        r = (pow(ax1 - cx, 2) + pow(ay1 - cy, 2));
    }
    dir = (buf[offset + 6] > 0) ? true : false;
    std::cout << "Arc: " << ax1 << ',' << ay1 << ',' << ax2 << ',' << ay2 << ','
              << cx << ',' << cy << ',' << (dir ? "CW" : "CCW") << std::endl;

    if (is_circle) {
        // center, radius, orientation=COUNTERCLOCKWISE
        circle = Circle_2(Point_2(cx, cy), FT(r));
        geo_type = GEO_CIRCLE;
        curve = Arrangement_Curve_2(circle);
    } else {
        r = CGAL::square(ax1 - cx) + CGAL::square(ay1 - cy);
        Circular_Circle_2 ccircle =
            Circular_Circle_2(Circular_Point_2(cx, cy), Circular_k::FT(r));
        // DOTO: orientation
        // center, arc_point, arc_point
        if (dir)
            arc = Circular_arc_2(
                ccircle, Circular_Arc_Point_2(Circular_Point_2(ax2, ay2)),
                Circular_Arc_Point_2(Circular_Point_2(ax1, ay1)));
        else
            arc = Circular_arc_2(
                ccircle, Circular_Arc_Point_2(Circular_Point_2(ax1, ay1)),
                Circular_Arc_Point_2(Circular_Point_2(ax2, ay2)));
        geo_type = GEO_ARC;
        std::cout << cx << ',' << cy << ',' << r << std::endl;
        std::cout << ax1 << ',' << ay1 << std::endl;
        std::cout << ax2 << ',' << ay2 << std::endl;
        circle = Circle_2(Point_2(cx, cy), FT(r));
        curve = Arrangement_Curve_2(circle);
        /*if (dir){
            Arrangement_Point_2 p1 = Arrangement_Point_2(FT(ax1), FT(ay1));
            Arrangement_Point_2 p2 = Arrangement_Point_2(FT(ax2), FT(ay2));
            auto p3 = Point_2(ax1, ay1) - Point_2(cx, cy);
            auto rr = sqrt(CGAL::to_double(CGAL::square(p3.x()) + CGAL::square(p3.y())));
            Circle_2 c = Circle_2(Point_2(cx, cy), rr);
            std::cout << CGAL::to_double(c.center().x()) << ',' << CGAL::to_double(c.center().y()) << ',';
            std::cout << CGAL::to_double(c.squared_radius()) << std::endl;
            std::cout << CGAL::to_double(p1.x()) << ',' << CGAL::to_double(p1.y()) << std::endl;
            std::cout << CGAL::to_double(p2.x()) << ',' << CGAL::to_double(p2.y()) << std::endl;
            curve = Arrangement_Curve_2(
                Point_2(cx, cy),
                rr, 
                CGAL::CLOCKWISE,
                p1,
                p2
            );
        }*/
        /*else
            curve = Arrangement_Curve_2(
                Circle_2(Point_2(cx, cy), r),
                Arrangement_Point_2(ax1, ay1),
                Arrangement_Point_2(ax2, ay2)
            );*/
    }
}

Bbox_2 GeometryArc::get_bbox()
{
    if (is_circle)
        return circle.bbox();
    else
        return arc.bbox();
}

void GeometryArc::dilate(Point_2 center, double dis)
{
    if (is_circle) {
        auto R = circle.squared_radius();
        circle = Circle_2(circle.center(), R + CGAL::square(dis) + 2 * sqrt(CGAL::to_double(R)) * dis);
        curve = Arrangement_Curve_2(circle);
    } else {
        auto R = arc.squared_radius();
        Circular_Circle_2 ccircle =
            Circular_Circle_2(arc.center(), R + CGAL::square(dis) + 2 * sqrt(CGAL::to_double(R)) * dis);

        double dx = CGAL::to_double(arc.source().x() - arc.center().x());
        double dy = CGAL::to_double(arc.source().y() - arc.center().y());
        double dl = sqrt(pow(dx, 2) + pow(dy, 2));
        Circular_Arc_Point_2 acp = Circular_Arc_Point_2(Circular_Point_2(
            CGAL::to_double(arc.source().x()) + dx * dis / dl,
            CGAL::to_double(arc.source().y()) + dy * dis / dl));

        dx = CGAL::to_double(arc.target().x() - arc.center().x());
        dy = CGAL::to_double(arc.target().y() - arc.center().y());
        dl = sqrt(pow(dx, 2) + pow(dy, 2));
        Circular_Arc_Point_2 acq = Circular_Arc_Point_2(Circular_Point_2(
            CGAL::to_double(arc.target().x()) + dx * dis / dl,
            CGAL::to_double(arc.target().y()) + dy * dis / dl));

        arc = Circular_arc_2(ccircle, acp, acq);

        circle = Circle_2(
            Point_2(
                CGAL::to_double(arc.center().x()), 
                CGAL::to_double(arc.center().y())), 
            CGAL::to_double(R) + CGAL::square(dis) + 2 * sqrt(CGAL::to_double(R)) * dis);
        curve = Arrangement_Curve_2(circle);
    }
}

void GeometryArc::check_collision(Geometry &g, std::vector<Point_2> &res) {}

Arrangement_Curve_2 GeometryArc::gen_curve(){
    return curve;
    if (is_circle)
        return Arrangement_Curve_2(circle);
    else{
        std::cout << "Constructing arc: " <<std::endl;
        std::cout << CGAL::to_double(arc.center().x()) << ',' << CGAL::to_double(arc.center().y()) << ',' << CGAL::to_double(arc.squared_radius()) << std::endl;
        std::cout << CGAL::to_double(arc.source().x()) << ',' << CGAL::to_double(arc.source().y()) << std::endl;
        std::cout << CGAL::to_double(arc.target().x()) << ',' << CGAL::to_double(arc.target().y()) << std::endl;
        return Arrangement_Curve_2(
                    Circle_2(Point_2(CGAL::to_double(arc.center().x()), 
                                     CGAL::to_double(arc.center().y())), 
                    CGAL::to_double(arc.squared_radius())),
                Arrangement_Point_2(CGAL::to_double(arc.source().x()), CGAL::to_double(arc.source().y())),
                Arrangement_Point_2(CGAL::to_double(arc.target().x()), CGAL::to_double(arc.target().y())));
    }
}

void GeometryArc::move(Vector_2 v) {}

void GeometryArc::draw(cv::Mat img,
                       const DrawConfig &dc,
                       const cv::Scalar color,
                       const int thickness,
                       const int lineType)
{
    double cx, cy;

    if (is_circle) {
        this->translate(cx, cy, circle.center(), dc);
        double r = sqrt(CGAL::to_double(circle.squared_radius())) * dc.scale;
        cv::circle(img, cv::Point(cx, cy), r, color, thickness, lineType);
    } else {
        double cx = CGAL::to_double(arc.center().x());
        double cy = CGAL::to_double(arc.center().y());
        double sx = CGAL::to_double(arc.source().x());
        double sy = CGAL::to_double(arc.source().y());
        double ex = CGAL::to_double(arc.target().x());
        double ey = CGAL::to_double(arc.target().y());

        double start_angle = atan2(sy - cy, sx - cx) * 180 / M_PI;
        double end_angle = atan2(ey - cy, ex - cx) * 180 / M_PI;
        if (end_angle < start_angle)
            end_angle += 360;
        Point_2 center = Point_2(cx, cy);

        double r = sqrt(CGAL::to_double(arc.squared_radius())) * dc.scale;

        this->translate(cx, cy, center, dc);
        cv::ellipse(img, cv::Point(cx, cy), cv::Size(r, r), 0, start_angle,
                    end_angle, color, thickness, lineType);
    }
}

Node::Node(const std::string &_node_type,
           const std::vector<std::string> &_geos_str)
{
    is_collision = false;
    if (_node_type.compare(0, 5, "drill") == 0)
        node_type = DRILL;
    else if (_node_type.compare(0, 3, "smd") == 0)
        node_type = SMD;
    else if (_node_type.compare(0, 3, "pth") == 0)
        node_type = PTH;
    else
        node_type = UNKNOWN;

    bbox = CGAL::Bbox_2();
    for (auto geo : _geos_str) {
        int start = geo.find(',');
        auto geo_type = geo.substr(0, start);

        // initialize according the type of geometry
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
        bbox += g->get_bbox();
    }
    center = Point_2((bbox.xmin() + bbox.xmax()) / 2,
                     (bbox.ymin() + bbox.ymax()) / 2);
    return bbox;
}

void Node::dilate(double dis)
{
    for (auto g : geometries) {
        g->dilate(center, dis);
    }
}

void Node::move(Vector_2 v)
{
    for (auto it = geometries.begin(); it < geometries.end(); it++) {
        // update position of all geometries
    }
}

void Node::print_bbox()
{
    std::cout << bbox.xmin() << ',' << bbox.ymin() << ',' << bbox.xmax() << ','
              << bbox.ymax() << std::endl;
}

bool Node::check_collision(const Node &n, cv::Mat img, const DrawConfig &dc)
{
    // collision check based on intersection of bounding boxes
    /*
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
    */
    // geometries recursive check
    
    CurveList curves;
    for (auto gi : geometries)
        curves.push_back(gi->gen_curve());
    for (auto gj : n.geometries) 
        curves.push_back(gj->gen_curve());

    // Compute all intersection points.

    std::list<Arrangement_Point_2> pts;
    CGAL::compute_intersection_points(curves.begin(), 
                                curves.end(),
                                std::back_inserter(pts),
                                false);

    if (pts.size() > 0){
        std::cout << "Found " << pts.size() << std::endl;
        for (auto p : pts){
            std::cout << "Collide " << CGAL::to_double(p.x()) << ',' <<CGAL::to_double(p.y()) <<std::endl;
            double ox = (CGAL::to_double(p.x()) - dc.s_xmin) * dc.t_w / dc.s_w;
            double oy = (CGAL::to_double(p.y()) - dc.s_ymin) * dc.t_h / dc.s_h;
            cv::circle(img, cv::Point(ox, oy), 5, cv::Scalar(0, 0, 255), 2);
        }
        return true;
    }
    else {
        std::cout << "Not found" <<std::endl;
        return false;
    }
}

void Node::draw(cv::Mat img, const DrawConfig &dc)
{
    for (auto g : geometries) {
        cv::Scalar color;
        // if (node_type == "drill")
        if (is_collision)
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