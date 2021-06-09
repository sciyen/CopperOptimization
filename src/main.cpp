#include <algorithm>  // for std::for_each
#include <iostream>   // for std::cout
#include <utility>    // for std::pair

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>

#include <iostream>
#include "geometry.h"
#include "load.h"
using namespace std;
using namespace boost;

typedef property<edge_weight_t, float> Weight;
typedef adjacency_list<vecS, vecS, undirectedS, no_property, Weight>
    UndirectedGraph;

int main()
{
    Load data = Load("../Testcase_txt/ProblemA.txt");
    for (auto node = data.nodes.begin(); node < data.nodes.end(); node++) {
        node->get_bbox();
        node->print_bbox();
    }

    UndirectedGraph undigraph(data.nodes.size());

    typename graph_traits<UndirectedGraph>::vertex_descriptor a, b;
    typedef typename UndirectedGraph::edge_property_type Weight;
    typename property_map<UndirectedGraph, edge_weight_t>::type weight =
        get(edge_weight, undigraph);
    typename graph_traits<UndirectedGraph>::edge_descriptor e1, e2;

    for (auto vi = data.nodes.begin(); vi < data.nodes.end(); vi++) {
        for (auto vj = vi + 1; vj < data.nodes.end(); vj++) {
            bool collision = vi->check_collision(*vj);
            if (collision) {
                int i = vi - data.nodes.begin();
                int j = vj - data.nodes.begin();
                a = vertex(i, undigraph);
                b = vertex(j, undigraph);
                add_edge(a, b, Weight(3.1), undigraph);
            }
        }
    }
    return 0;
}