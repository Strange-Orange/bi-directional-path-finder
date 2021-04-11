#include "parseData.h"
#include "vertex.h"
#include "edge.h"
#include "common.h"

#include <unordered_map>
#include <vector>
#include <utility>
#include <fstream>
#include <algorithm>

#include <cmath>
#include <limits.h>
#include <stdint.h>

#define DIR_NORTH 0x01
#define DIR_EAST 0x02
#define DIR_SOUTH 0x04
#define DIR_WEST 0x08

const int g_SEGMENTS = 10;
Bounds g_geoBounds;

adjacencyList parse_data_csv(const char* p_filepath)
{
    std::vector<Vertex> l_latvec;
    std::vector<Vertex> l_lngvec;

    std::string l_name;
    std::string l_lat;
    std::string l_latd;
    std::string l_lng;
    std::string l_lngd;
    std::string l_pop;

    std::fstream l_file(p_filepath);
    // Skip first line
    std::getline(l_file, l_name);
    // City name, latitude, longitude
    while (std::getline(l_file, l_name, ','))
    {
        // Get the latitude and longitude as integers can divided by 10,000 if decimal required
        std::getline(l_file, l_lat, '.');
        std::getline(l_file, l_latd, ',');
        std::getline(l_file, l_lng, '.');
        std::getline(l_file, l_lngd, ',');

        int l_vlat = (stoi(l_lat + l_latd));
        int l_vlng = (stoi(l_lng + l_lngd));
        Vertex l_v(l_vlat, l_vlng, l_name);
        l_latvec.emplace_back(l_v);
        l_lngvec.emplace_back(l_v);
        // Jump to the next line
        std::getline(l_file, l_name);
    }
    std::sort(l_latvec.begin(), l_latvec.end(), [](const Vertex& lhs, const Vertex& rhs){return lhs.get_lat() < rhs.get_lat();});
    std::sort(l_lngvec.begin(), l_lngvec.end(), [](const Vertex& lhs, const Vertex& rhs){return lhs.get_lng() < rhs.get_lng();});

    // Update the bounds of the area to be mapped
    g_geoBounds.m_west = l_lngvec.at(0).get_lng();
    g_geoBounds.m_east = l_lngvec.at(l_lngvec.size() - 1).get_lng();
    g_geoBounds.m_south = l_latvec.at(0).get_lat();  
    g_geoBounds.m_north = l_latvec.at(l_latvec.size() - 1).get_lat();

    adjacencyList l_adj;
    create_adjacency_list(l_adj, l_latvec, l_lngvec);
    return l_adj;
}

// Search vector for Vertex using longitude
bool binary_search_longitude(const std::vector<Vertex>& p_v, double p_item)
{
    int l_low = 0, l_high = p_v.size() - 1;
    while (l_low <= l_high)
    {
        int l_mid = (l_low + l_high) / 2;
        if (p_v.at(l_mid).get_lng() == p_item)
            return true;
        else if (p_v.at(l_mid).get_lng() < p_item)
            l_low = l_mid + 1;
        else
            l_high = l_mid - 1;
    }
    return false;
}

bool binary_search_latitude(const std::vector<Vertex>& p_v, double p_item)
{
    int l_low = 0, l_high = p_v.size() - 1;
    while (l_low <= l_high)
    {
        int l_mid = (l_low + l_high) / 2;
        if (p_v.at(l_mid).get_lat() == p_item)
            return true;
        else if (p_v.at(l_mid).get_lat() < p_item)
            l_low = l_mid + 1;
        else
            l_high = l_mid - 1;
    }
    return false;
}

inline int calc_distance(const Vertex& p_source, const Vertex& p_dest)
{
    int l_x = abs(p_source.get_lng() - p_dest.get_lng());
    int l_y = abs(p_source.get_lng() - p_dest.get_lng());
    return sqrt((l_x * l_x) + (l_y * l_y));
}

void create_row_segments(const std::vector<Vertex>& p_lats, vectorVertex2d& o_segments)
{
    int l_range = g_geoBounds.m_north - g_geoBounds.m_south;
    int l_dist = abs(ceil(l_range / g_SEGMENTS));
    int l_bound = g_geoBounds.m_south + l_dist;
    size_t l_segIndex = 0;
    for (size_t i = 0; i < g_SEGMENTS - 1; i++)
    {
        std::vector<Vertex> l_verticesInSeg;
        while ((l_segIndex < p_lats.size()) && (p_lats.at(l_segIndex).get_lat() < l_bound + 1))
            l_verticesInSeg.push_back(p_lats.at(l_segIndex++));

        o_segments.push_back(l_verticesInSeg);
        l_bound += l_dist;
    }
    // Add the last vertices in case of a rounding error
    std::vector<Vertex> l_last;
    while (l_segIndex < p_lats.size())
        l_last.push_back(p_lats.at(l_segIndex++));
    o_segments.push_back(l_last);
}

void create_col_segments(const std::vector<Vertex>& p_lngs, vectorVertex2d& o_segments)
{
    int l_range = g_geoBounds.m_east - g_geoBounds.m_west;
    int l_dist = abs(ceil(l_range / g_SEGMENTS));
    int l_bound = g_geoBounds.m_west + l_dist;
    size_t l_segIndex = 0;
    for (size_t i = 0; i < g_SEGMENTS - 1; i++)
    {
        std::vector<Vertex> l_verticesInSeg;
        while ((l_segIndex < p_lngs.size()) && (p_lngs.at(l_segIndex).get_lng() < l_bound + 1))
            l_verticesInSeg.push_back(p_lngs.at(l_segIndex++));
        o_segments.push_back(l_verticesInSeg);
        l_bound += l_dist;
    }
    // Add the last vertices in case of a rounding error
    std::vector<Vertex> l_last;
    while (l_segIndex < p_lngs.size())
        l_last.push_back(p_lngs.at(l_segIndex++));
    o_segments.push_back(l_last);

}

void create_grid_segments(const vectorVertex2d& p_rows, const  vectorVertex2d& p_cols,  vectorVertex2d& o_grid, std::vector<Segment>& o_segmentInfo)
{
    size_t l_segmentIndex = 0;
    for (const std::vector<Vertex>& l_row: p_rows)
    {
        for (const std::vector<Vertex>& l_col: p_cols)
        {
            std::vector<Vertex> l_segmentVertices;
            Segment l_segment{l_segmentIndex, -1, -1, -1, -1};
            for (const Vertex& l_v: l_col)
            {
                if (binary_search_latitude(l_row, l_v.get_lat()))
                {
                    // Update the segment info, if there is only 1 vertex in a segment then it is the "furthest" in all directions
                    if (l_segment.m_northIndex == -1 || l_v.get_lat() > l_segmentVertices.at(l_segment.m_northIndex).get_lat())
                        l_segment.m_northIndex = l_segmentVertices.size();

                    if (l_segment.m_eastIndex == -1 || l_v.get_lng() > l_segmentVertices.at(l_segment.m_eastIndex).get_lng())
                        l_segment.m_eastIndex = l_segmentVertices.size();

                    if (l_segment.m_southIndex == -1 || l_v.get_lat() < l_segmentVertices.at(l_segment.m_southIndex).get_lat())
                        l_segment.m_southIndex = l_segmentVertices.size();

                    if (l_segment.m_westIndex == -1 || l_v.get_lng() < l_segmentVertices.at(l_segment.m_westIndex).get_lng())
                        l_segment.m_westIndex = l_segmentVertices.size();

                    l_segmentVertices.push_back(l_v);
                }
            }
            // Avoid making a copy
            o_grid.push_back(std::move(l_segmentVertices));
            o_segmentInfo.at(l_segmentIndex++) = std::move(l_segment);
        }
    }
}

uint8_t neighbour_direction(const Vertex& p_current, const Vertex& p_viewing)
{
    // Get the direction of the current viewing vertex relative to the current vertex

    int l_xLen = p_viewing.get_lng() - p_current.get_lng();
    int l_yLen = p_viewing.get_lat() - p_current.get_lat();
    double l_angle = atan2(l_yLen, l_xLen);
    if (l_angle <= 2.36 && l_angle >= 0.79)
        return DIR_NORTH;
    else if (l_angle < 0.79 && l_angle >= -0.79)
        return DIR_EAST;
    else if (l_angle > 2.36 && l_angle <= 3.39)
        return DIR_WEST;
    else
        return DIR_SOUTH;
}

void find_neighbours(const Vertex& p_current, const std::vector<Vertex>& p_segment, std::vector<Edge>& o_n)
{
    int l_northDist = INT_MAX;
    int l_eastDist = INT_MAX;
    int l_southDist = INT_MAX;
    int l_westDist = INT_MAX;
    // Order of indices is north, east, south west 
    int l_indices[4];
    // Initialize all the values to -1
    for (int i = 0; i < 4; i++)
        l_indices[i] = -1;
    
    int l_viewingIndex = 0;
    for (const Vertex& l_viewing: p_segment)
    {
        if (l_viewing != p_current)
        {
            uint8_t l_direction = neighbour_direction(p_current, l_viewing);
            if (l_direction & DIR_NORTH)
            {
                if (calc_distance(p_current, l_viewing) < l_northDist)
                    l_indices[0] = l_viewingIndex;
            }
            else if (l_direction & DIR_EAST)
            {
                if (calc_distance(p_current, l_viewing) < l_eastDist)
                    l_indices[1] = l_viewingIndex;
            }
            else if (l_direction & DIR_SOUTH)
            {
                if (calc_distance(p_current, l_viewing) < l_southDist)
                    l_indices[2] = l_viewingIndex;
            }
            else 
            {
                if (calc_distance(p_current, l_viewing) < l_westDist)
                    l_indices[3] = l_viewingIndex;
            }
        }
        l_viewingIndex++;
    }

    // Add the neighours as edges to the output vector
    for (int i = 0; i < 4; i++)
    {
        if (l_indices[i] != -1)
            o_n.push_back({p_current, p_segment.at(l_indices[i])});
    }
}

// Connect the vertices in a segment, connect each vertex to neighbour in the north, east, west and south directions
void connect_vertices(adjacencyList& p_adj, const vectorVertex2d& p_grid)
{
    for (const std::vector<Vertex>& l_segment: p_grid)
    {
        for (size_t l_current = 0; l_current < l_segment.size(); l_current++)
        {
            const Vertex& l_v = l_segment.at(l_current);
            p_adj[l_v];
            std::vector<Edge> l_n;
            find_neighbours(l_v, l_segment, l_n);
            p_adj.at(l_v) = std::move(l_n);
        }
    }
}

void connect_grid(adjacencyList& p_adj, const vectorVertex2d& p_grid, const std::vector<Segment>& p_segmentInfo)
{
    for (size_t i = 0; i < p_grid.size() - 1; i++)
    {
        // Connect segments on the x axis
        if (p_grid.at(i).size() > 0)
        {
            const Vertex& l_eastPoint = p_grid.at(i).at(p_segmentInfo.at(i).m_eastIndex);
            int l_lngViewingSegment = i;
            while (++l_lngViewingSegment % 10 != 0)
            {
                if (p_segmentInfo.at(l_lngViewingSegment).m_westIndex != -1)
                {
                    const Vertex& l_neighbourWestPoint = p_grid.at(l_lngViewingSegment).at(p_segmentInfo.at(l_lngViewingSegment).m_westIndex);
                    p_adj.at(l_eastPoint).push_back({l_eastPoint, l_neighbourWestPoint});
                    p_adj.at(l_neighbourWestPoint).push_back({l_neighbourWestPoint, l_eastPoint});
                    break;
                }
            }
            // Connect segments on the y axis
            const Vertex& l_northPoint = p_grid.at(i).at(p_segmentInfo.at(i).m_northIndex);
            int l_latViewingSegment = i + g_SEGMENTS;
            while (l_latViewingSegment < (g_SEGMENTS * g_SEGMENTS) - g_SEGMENTS)
            {
                if (p_segmentInfo.at(l_latViewingSegment).m_southIndex != -1)
                {
                    const Vertex& l_neighbourSouthPoint = p_grid.at(l_latViewingSegment).at(p_segmentInfo.at(l_latViewingSegment).m_southIndex);
                    p_adj.at(l_northPoint).push_back({l_northPoint, l_neighbourSouthPoint});
                    p_adj.at(l_neighbourSouthPoint).push_back({l_neighbourSouthPoint, l_northPoint});
                    break;
                }
                l_latViewingSegment += g_SEGMENTS;
            }   
        }
    }
}
// For each Vertex (City) have a most 4 edges, reaching to the closest in North, East, South and West directions
void create_adjacency_list(adjacencyList& p_adj, const std::vector<Vertex>& p_lats, const std::vector<Vertex>& p_lngs)
{
    vectorVertex2d l_rows;
    vectorVertex2d l_cols;
    create_row_segments(p_lats, l_rows);
    create_col_segments(p_lngs, l_cols);
    vectorVertex2d l_grid;
    std::vector<Segment> l_segmentInfo(g_SEGMENTS * g_SEGMENTS);
    create_grid_segments(l_rows, l_cols, l_grid, l_segmentInfo);

    connect_vertices(p_adj, l_grid);
    connect_grid(p_adj, l_grid, l_segmentInfo);
}

// Get the bound of the area parsed, a file must be parsed before and called to this function, otherwise all members will be 0
Bounds& get_bounds()
{
    return g_geoBounds;
}
