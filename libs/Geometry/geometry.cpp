#include "geometry.h"

Geometry::Geometry() {}

Bbox_2 Geometry::get_bbox()
{
    return CGAL::Bbox_2();
}

int Geometry::parse(std::string s, float *params)
{
    int i, pre = 0;
    size_t p = 0;
    for (i = 0; (p = s.find(',', pre)) != std::string::npos; i++)
    {
        params[i] = std::stof(s.substr(pre, p - pre));
        pre = p + 1;
    }
    std::string last = s.substr(pre);
    try
    {
        params[i] = std::stof(last);
    }
    catch (const std::invalid_argument &ia)
    {
        // compare CW: 1, CCW: -5140
        params[i] = (last.compare("CW") > 0) ? 1 : -1;
    }
    return i;
}

GeometryLine::GeometryLine(const std::string &geo) : Geometry()
{
    add_feature(geo);
    bbox = get_bbox();
};

void GeometryLine::add_feature(const std::string &s)
{
    float x1, y1, x2, y2;
    float buf[4];
    parse(s, buf);
    x1 = buf[0];
    y1 = buf[1];
    x2 = buf[2];
    y2 = buf[3];
    std::cout << "line: " << x1 << ',' << y1 << ',' << x2 << ',' << y2 << std::endl;

    seg = Segment_2(Point_2(x1, y1), Point_2(x2, y2));
}

Bbox_2 GeometryLine::get_bbox()
{
    std::cout << "arc" << std::endl;
    return seg.bbox();

    GeometryArc::GeometryArc(const std::string &geo) : Geometry()
    {
        is_circle = false;
        add_feature(geo);
        bbox = get_bbox();
    };

    void GeometryArc::add_feature(const std::string &s)
    {
        float ax1, ay1, ax2, ay2, cx, cy, r;
        float buf[8];
        int num = parse(s, buf);
        int offset = 0;
        ax1 = buf[0];
        ay1 = buf[1];
        ax2 = buf[2];
        ay2 = buf[3];
        cx = buf[4];
        cy = buf[5];
        if (num == 7)
        {
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
        else
            // DOTO: orientation
            // center, arc_point, arc_point
            arc = Circular_arc_2(
                Circular_Point_2(cx, cy),
                Circular_Arc_Point_2(Circular_Point_2(ax1, ay1)),
                Circular_Arc_Point_2(Circular_Point_2(ax2, ay2)));
    }

    Bbox_2 GeometryArc::get_bbox()
    {
        std::cout << "arc" << std::endl;

        if (is_circle)
            return circle.bbox();
        else
            return arc.bbox();
    }

    Node::Node(const std::string &_node_type, const std::vector<std::string> &_geos_str)
    {
        node_type = _node_type;
        bbox = CGAL::Bbox_2();
        for (auto geo : _geos_str)
        {
            int start = geo.find(',');
            auto geo_type = geo.substr(0, start);

            // initialize according the type of geometry
            Geometry *g;
            if (geo_type == "line")
                g = new GeometryLine(geo.substr(start + 1));
            else if (geo_type == "arc")
                g = new GeometryArc(geo.substr(start + 1));

            geometries.push_back(g);
        }
    }

    Bbox_2 Node::get_bbox()
    {
        std::cout << "type: " << node_type << std::endl;
        for (auto g : geometries)
        {
            //Bbox_2 gb = g->get_bbox();
            //std::cout << gb.xmin() << std::endl;
            //bbox += gb;
            bbox += g->bbox;
        }
        center = Point_2((bbox.xmin() + bbox.xmax()) / 2, (bbox.ymin() + bbox.ymax()) / 2);
        return bbox;
    }

    void Node::print_bbox()
    {
        std::cout << bbox.xmin() << ',' << bbox.ymin() << ',' << bbox.xmax() << ',' << bbox.ymax() << std::endl;
    }

    bool Node::check_collision(const Node &n)
    {
        Iso_rectangle_2 rec0(Exact_Point_2(bbox.xmin(), bbox.ymin()), Exact_Point_2(bbox.xmax(), bbox.ymax()));
        Iso_rectangle_2 rec1(Exact_Point_2(n.bbox.xmin(), n.bbox.ymin()), Exact_Point_2(n.bbox.xmax(), n.bbox.ymax()));

        CGAL::cpp11::result_of<Intersect_2(Iso_rectangle_2, Iso_rectangle_2)>::type
            result = CGAL::intersection(rec0, rec1);
        if (result)
        {
            const Iso_rectangle_2 *s = boost::get<Iso_rectangle_2>(&*result);
            return true;
        }
        return false;
    }

    Node::~Node()
    {
        //for (auto it = geometries.begin(); it < geometries.end(); it++)
        //    delete *it;
    }