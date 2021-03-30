#ifndef _PARSE_DATA_H
#define _PARSE_DATA_H

#include "vertex.h"
#include "edge.h"
#include "common.h"

#include <vector>
#include <unordered_map>

typedef std::vector<std::vector<Vertex>> vectorVertex2d;

struct Bounds
{
    double m_east = 0;
    double m_west = 0;
    double m_south = 0;
    double m_north = 0;
    std::unordered_map<std::string, Vertex> m_cities;
};

adjacencyList parse_data_csv(const char* p_filepath);
bool binary_search_longitude(const std::vector<Vertex>& p_v, double p_item);
bool binary_search_latitude(const std::vector<Vertex>& p_v, double p_item);
inline int calc_distance(const Vertex& p_source, const Vertex& p_dest);
void create_row_segments(const std::vector<Vertex>& p_lats, vectorVertex2d& o_segments);
void create_col_segments(const std::vector<Vertex>& p_lngs, vectorVertex2d& o_segments);
void create_grid_segments(const vectorVertex2d& p_rows, const vectorVertex2d& p_cols, vectorVertex2d& o_grid);
void create_adjacency_list(adjacencyList& p_adj, const std::vector<Vertex>& p_lats, const std::vector<Vertex>& p_lngs);
Bounds& get_bounds();

#endif