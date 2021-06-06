#include "geometry.h"

Geometry::Geometry() {}

int Geometry::parse(string s, float *params)
{
    int i, pre = 0;
    size_t p = 0;
    for (i = 0; (p = s.find(',', pre)) != string::npos; i++) {
        params[i] = stof(s.substr(pre, p - pre));
        pre = p + 1;
    }
    string last = s.substr(pre);
    try {
        params[i] = stof(last);
    } catch (const std::invalid_argument &ia) {
        // compare CW: 1, CCW: -5140
        params[i] = (last.compare("CW") > 0) ? 1 : 0;
    }
    return i;
}

GeometryLine::GeometryLine(const string &geo) : Geometry()
{
    add_feature(geo);
};

void GeometryLine::add_feature(const string &s)
{
    float buf[4];
    parse(s, buf);
    x1 = buf[0];
    y1 = buf[1];
    x2 = buf[2];
    y2 = buf[3];
    cout << "line: " << x1 << ',' << y1 << ',' << x2 << ',' << y2 << endl;
}

GeometryArc::GeometryArc(const string &geo) : Geometry()
{
    add_feature(geo);
};

void GeometryArc::add_feature(const string &s)
{
    float buf[8];
    int num = parse(s, buf);
    int offset = 0;
    if (num == 7)
        offset = 1;
    ax1 = buf[offset + 0];
    ay1 = buf[offset + 1];
    ax2 = buf[offset + 2];
    ay2 = buf[offset + 3];
    cx = buf[offset + 4];
    cy = buf[offset + 5];
    dir = (buf[offset + 6] > 0) ? true : false;
    cout << "Arc: " << ax1 << ',' << ay1 << ',' << ax2 << ',' << ay2 << ','
         << cx << ',' << cy << ',' << (dir ? "CW" : "CCW") << endl;
}

Node::Node(const string &_node_type, const vector<string> &_geos_str)
{
    node_type = _node_type;
    for (auto geo : _geos_str) {
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