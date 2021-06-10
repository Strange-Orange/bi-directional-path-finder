#ifndef _PARSE_DATA_H
#define _PARSE_DATA_H

#include "vertex.h"
#include "edge.h"
#include "common.h"

#include <vector>
#include <utility>

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

// Try changing the size of the segments depending on the number of vertices inside a segment

adjacencyList parse_data_csv(const char* p_filepath);
int binary_search_latitude(const std::vector<Vertex>& p_vertices, const Vertex& p_item);
inline int calc_distance(const Vertex& p_source, const Vertex& p_dest);
inline bool segment_in_bounds(int p_index, int p_dirIndex);
inline bool opposite_directions(int p_index, uint8_t p_directions);
uint8_t direction_from_segment(int p_center, int p_viewing);
inline uint8_t invert_directions(uint8_t p_directions);
void create_row_segments(const std::vector<Vertex>& p_lats, vectorVertex2d& o_segments);
void create_col_segments(const std::vector<Vertex>& p_lngs, vectorVertex2d& o_segments);
void create_grid_segments(const vectorVertex2d& p_rows, const  vectorVertex2d& p_cols,  vectorVertex2d& o_grid, std::vector<Segment>& o_segmentInfo);
uint8_t neighbour_direction(const Vertex& p_current, const Vertex& p_viewing);
void find_neighbours(const Vertex& p_current, const std::vector<Vertex>& p_segment, std::vector<Edge>& o_n);
void connect_vertices(adjacencyList& p_adj, const vectorVertex2d& p_grid);
std::pair<Vertex, Vertex> vertices_to_connect_segments(const vectorVertex2d& p_grid, int p_src, const std::pair<size_t, uint8_t>& p_dest, const std::vector<Segment>& p_segmentInfo);
void connect_grid(adjacencyList& p_adj, const vectorVertex2d& p_grid, const std::vector<Segment>& p_segmentInfo);
void create_adjacency_list(adjacencyList& p_adj, const std::vector<Vertex>& p_lats, const std::vector<Vertex>& p_lngs);
Bounds& get_bounds();

#endif
