#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <iostream>
#include <string>
#include <vector>
using namespace std;

/* Base class of simple 2D geometries */
class Geometry
{
protected:
    int parse(string s, float *params);

public:
    Geometry();
    virtual void add_feature(const string &s) = 0;
};

class GeometryLine : public Geometry
{
private:
public:
    float x1, y1, x2, y2;
    GeometryLine(const string &s);
    void add_feature(const string &s);
};

class GeometryArc : public Geometry
{
private:
public:
    float ax1, ay1, ax2, ay2, cx, cy, r;
    bool dir; // true for cw, false for ccw
    GeometryArc(const string &s);
    void add_feature(const string &s);
};

class Node
{
private:
public:
    string node_type;
    vector<Geometry *> geometries;

    /* Constructor for Node
     * @Params:
     *      node_type: "drill", "pth", "smd"
     *      geos_str:  geometries descriptor.
     */
    Node(const string &node_type, const vector<string> &geos_str);
};

#endif