#include <algorithm>  // for std::for_each
#include <iostream>   // for std::cout
#include <utility>    // for std::pair

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>

#include <iostream>
#include <opencv2/opencv.hpp>
#include "geometry.h"
#include "load.h"
#include "visualization.h"

using namespace boost;

typedef property<edge_weight_t, double> Weight;
typedef adjacency_list<vecS, vecS, undirectedS, no_property, Weight>
    UndirectedGraph;

int main(int argc, char *argv[])
{
    // Read filename from runtime parameters
    if (argc != 2) {
        std::cout << "This program require an input of input data(txt file)"
                  << std::endl;
        return 0;
    }

    // Load data from txt file
    Load data = Load(argv[1]);

    // initial plot
    data.get_bbox();
    Visualization fig = Visualization(data, "pad");
    fig.show(data);
    fig.imwrite("./output/original.jpg");

    // Generate detection zone
    for (auto vi = data.nodes.begin(); vi < data.nodes.end(); vi++) {
        vi->dilate(data.config.mingap / 2);
    }
    data.get_bbox();

    /* Graph construction */
    UndirectedGraph undigraph(data.nodes.size());

    typename graph_traits<UndirectedGraph>::vertex_descriptor a, b;
    typedef typename UndirectedGraph::edge_property_type Weight;
    typename property_map<UndirectedGraph, edge_weight_t>::type weight =
        get(edge_weight, undigraph);
    typename graph_traits<UndirectedGraph>::edge_descriptor e1, e2;

    bool game_alive = true;
    int iteration_count = 0;
    while (game_alive && (iteration_count < 300)) {
        fig.reset();

        // Clear all neighbors
        for (auto vi = data.nodes.begin(); vi < data.nodes.end(); vi++) {
            vi->collision_nbr.clear();
            vi->get_bbox();
        }

        // check the collision states
        bool any_collision = false;
        for (auto vi = data.nodes.begin(); vi < data.nodes.end(); vi++) {
            for (auto vj = vi + 1; vj < data.nodes.end(); vj++) {
                if (vi->node_type == DRILL || vj->node_type == DRILL) {
                    // collision with drill is not concerned
                    continue;
                }

                bool collision = vi->check_collision(*vj, fig.img, fig.config);
                if (collision) {
                    vi->is_collision = true;
                    vj->is_collision = true;
                    vi->collision_nbr.push_back(vj - data.nodes.begin());
                    vj->collision_nbr.push_back(vi - data.nodes.begin());
                    int i = vi - data.nodes.begin();
                    int j = vj - data.nodes.begin();
                    a = vertex(i, undigraph);
                    b = vertex(j, undigraph);
                    add_edge(a, b, Weight(3.1), undigraph);
                    any_collision = true;
                }
            }
        }
        if (!any_collision)
            game_alive = false;

        // forces calculation
        for (auto vi = data.nodes.begin(); vi < data.nodes.end(); vi++) {
            vi->force = Vector_2(0, 0);
            // repulsive forces
            for (auto i : vi->collision_nbr) {
                // the collision node
                auto v_n = data.nodes.begin() + i;
                vi->force += vi->center - v_n->center;
            }
            // alpha
            vi->force *= 0.5;

            // attractive force
            vi->force += 0.5 * (vi->origin - vi->center);
        }

        // position updating
        for (auto vi = data.nodes.begin(); vi < data.nodes.end(); vi++) {
            vi->move(vi->force / 50);
        }

        iteration_count++;

        fig.show(data);
        fig.draw();
        fig.imwrite(std::string("./output/") + std::to_string(iteration_count) +
                    std::string(".jpg"));
    }
    fig.imwrite("./output/result.jpg");
    return 0;
}