#include "load.h"

Load::Load(std::string fname)
{
    filename = fname;
    read();
}

void Load::read()
{
    std::cout << "Reading " << filename << std::endl;
    std::ifstream infile(filename);

    std::string line;
    std::string node_type;
    std::vector<std::string> geo_buf;
    while (getline(infile, line)) {
        size_t pos = 0;
        std::string s;
        if ((pos = line.find(',')) == std::string::npos) {
            // drill, pth, smd

            // deal with previous geo parameters
            if (geo_buf.size() > 0) {
                nodes.push_back(Node(node_type, geo_buf));
                geo_buf.clear();
            }

            // new geo type
            node_type = line;
        } else {
            std::string token = line.substr(0, pos);

            // deal with header
            size_t sec_pos = pos;
            if ((sec_pos = line.find(',', pos + 1)) == std::string::npos) {
                double value = stof(line.substr(pos + 1));
                if (token == "mingap")
                    config.mingap = value;
                else if (token == "minwidth")
                    config.minwidth = value;
                else if (token == "geometry")
                    config.geometry = value;
                std::cout << "settings: " << token << ", " << value
                          << std::endl;
            }

            // deal with parameters of a geometric
            else {
                geo_buf.push_back(line);
            }
        }
    }

    // the last geo
    nodes.push_back(Node(node_type, geo_buf));
}

Bbox_2 Load::get_bbox()
{
    bbox = CGAL::Bbox_2();
    for (auto node = nodes.begin(); node < nodes.end(); node++) {
        bbox += node->get_bbox();
    }
    return bbox;
}