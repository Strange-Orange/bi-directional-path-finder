#ifndef _PARSE_DATA_H
#define _PARSE_DATA_H

#include "vertex.h"
#include "edge.h"
#include "common.h"

#include <vector>
#include <unordered_map>

#include <stdint.h>

typedef std::vector<std::vector<Vertex>> vectorVertex2d;

struct Bounds
{
    int m_east = 0;
    int m_west = 0;
    int m_south = 0;
    int m_north = 0;
};

struct Segment
{
    size_t m_segmentIndex;
    int m_northIndex;
    int m_eastIndex;
    int m_southIndex;
    int m_westIndex;
};

adjacencyList parse_data_csv(const char* p_filepath);
bool binary_search_longitude(const std::vector<Vertex>& p_v, double p_item);
bool binary_search_latitude(const std::vector<Vertex>& p_v, double p_item);
inline int calc_distance(const Vertex& p_source, const Vertex& p_dest);
void create_row_segments(const std::vector<Vertex>& p_lats, vectorVertex2d& o_segments);
void create_col_segments(const std::vector<Vertex>& p_lngs, vectorVertex2d& o_segments);
void create_grid_segments(const vectorVertex2d& p_rows, const  vectorVertex2d& p_cols,  vectorVertex2d& o_grid, std::vector<Segment>& o_segmentInfo);
// int find_neighbour(int p_direction, int p_currentIndex, const std::vector<Vertex>& p_vertices);
uint8_t neighbour_direction(const Vertex& p_current, const Vertex& p_viewing);
void find_neighbours(const Vertex& p_current, const std::vector<Vertex>& p_segment, std::vector<Edge>& o_n);
void connect_vertices(adjacencyList& p_adj, const vectorVertex2d& p_grid);
void connect_grid(adjacencyList& p_adj, const vectorVertex2d& p_grid, const std::vector<Segment>& p_segmentInfo);
void create_adjacency_list(adjacencyList& p_adj, const std::vector<Vertex>& p_lats, const std::vector<Vertex>& p_lngs);
Bounds& get_bounds();

#endif