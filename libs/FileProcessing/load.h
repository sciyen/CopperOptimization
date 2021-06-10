#ifndef LOAD_H
#define LOAD_H

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "geometry.h"

struct DataConfig
{
    double mingap;
    double minwidth;
    double geometry;
};

class Load
{
private:
    std::string filename;

    /* Parse the file */
    void read();

public:
    CGAL::Bbox_2 bbox;
    DataConfig config;
    std::string node_type;
    std::vector<Node> nodes;

    /* Load from txt file */
    Load(std::string fname);

    /* Calculate maximal bbox */
    Bbox_2 get_bbox();
};

#endif
