#ifndef LOAD_H
#define LOAD_H

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "geometry.h"
using namespace std;

struct DataConfig {
    float mingap;
    float minwidth;
    float geometry;
};

class Load
{
private:
    string filename;

    /* Parse the file */
    void read();

public:
    DataConfig config;
    string node_type;
    vector<Node> nodes;

    /* Load from txt file */
    Load(string fname);
};

#endif
