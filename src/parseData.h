#ifndef _PARSE_DATA_H
#define _PARSE_DATA_H

#include "vertex.h"
#include "edge.h"
#include "common.h"

#include <vector>
#include <unordered_map>

struct Bounds
{
    double m_east = 0;
    double m_west = 0;
    double m_south = 0;
    double m_north = 0;
    std::unordered_map<std::string, Vertex> m_cities;
};

adjacencyList parse_data_csv(const char* p_filepath);
int binary_search_longitude(const std::vector<Vertex>& p_v, double p_item);
bool duplicate_neighbour(const std::vector<Edge>& p_n, const std::string& p_newN);
int get_east_neighbour(const std::vector<Edge>& p_n, const std::vector<Vertex>& p_lngs, size_t p_lngIndex);
int get_west_neighbour(const std::vector<Edge>& p_n, const std::vector<Vertex>& p_lngs, int p_lngIndex);
void create_adjacency_list(adjacencyList& p_adj, const std::vector<Vertex>& p_lats, const std::vector<Vertex>& p_lngs);
Bounds get_bounds();

#endif