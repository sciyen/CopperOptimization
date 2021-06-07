#include "load.h"

Load::Load(string fname)
{
    filename = fname;
    read();
}

void Load::read()
{
    cout << "Reading " << filename << endl;
    ifstream infile(filename);

    string line;
    string node_type;
    vector<string> geo_buf;
    while (getline(infile, line)) {
        size_t pos = 0;
        string s;
        if ((pos = line.find(',')) == string::npos) {
            // drill, pth, smd

            // deal with previous geo parameters
            if (geo_buf.size() > 0) {
                nodes.push_back(Node(node_type, geo_buf));
                geo_buf.clear();
            }

            // new geo type
            node_type = line;
        } else {
            string token = line.substr(0, pos);

            // deal with header
            size_t sec_pos = pos;
            if ((sec_pos = line.find(',', pos)) == string::npos) {
                float value = stof(line.substr(pos + 1));
                if (token == "mingap")
                    config.mingap = value;
                else if (token == "minwidth")
                    config.minwidth = value;
                else if (token == "geometry")
                    config.geometry = value;
                cout << "settings: " << token << ", " << value << endl;
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
