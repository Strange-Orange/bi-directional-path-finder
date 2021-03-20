#ifndef _PARSE_DATA_H
#define _PARSE_DATA_H

#include "vertex.h"
#include "edge.h"
#include "common.h"

#include <vector>

struct Bounds
{
    double m_east = 0;
    double m_west = 0;
    double m_south = 0;
    double m_north = 0;
};

adjacencyList parse_data_csv(const char* p_filepath);
int binary_search_latitude(const std::vector<Vertex>& p_v, double p_item);
void create_adjacency_list(adjacencyList& p_adj, const std::vector<Vertex>& p_lats, const std::vector<Vertex>& p_lngs);
Bounds get_bounds();

#endif