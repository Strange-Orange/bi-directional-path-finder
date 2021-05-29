#include "parseData.h"
#include "vertex.h"
#include "edge.h"
#include "common.h"

#include <iostream>
#include <string>
#include <unordered_set>
#include <array>
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
#define DIR_ALL 0xF

#define DIRECTIONS 8

const int g_SEGMENTS = 50;
const int g_TOTALSEGMENTS = g_SEGMENTS * g_SEGMENTS;
// Values to be added to move between segments
// Ordering is North, east, south, west, north east, south east, south west and north west
const std::array<int, DIRECTIONS> g_touchingSegments {g_SEGMENTS, 1, -g_SEGMENTS, -1, g_SEGMENTS + 1, -g_SEGMENTS + 1, -g_SEGMENTS - 1, g_SEGMENTS - 1};
const std::array<uint8_t, DIRECTIONS> g_dirMasks 
{
    DIR_NORTH,
    DIR_EAST,
    DIR_SOUTH,
    DIR_WEST, 
    DIR_NORTH | DIR_EAST,
    DIR_SOUTH | DIR_EAST,
    DIR_SOUTH | DIR_WEST,
    DIR_NORTH | DIR_WEST
};

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

int binary_search_latitude(const std::vector<Vertex>& p_vertices, const Vertex& p_item)
{
    int l_low = 0, l_high = p_vertices.size() - 1;
    while (l_low <= l_high)
    {
        int l_mid = (l_low + l_high) / 2;
        if (p_vertices.at(l_mid).get_lat() == p_item.get_lat())
            return l_mid;
        else if (p_vertices.at(l_mid).get_lat() < p_item.get_lat())
            l_low = l_mid + 1;
        else
            l_high = l_mid - 1;
    }
    return -1;
}

inline int calc_distance(const Vertex& p_source, const Vertex& p_dest)
{
    int l_x = abs(p_source.get_lng() - p_dest.get_lng());
    int l_y = abs(p_source.get_lng() - p_dest.get_lng());
    return sqrt((l_x * l_x) + (l_y * l_y));
}

// Make sure a segment and a movement do not move out of bounds of the grid
inline bool segment_in_bounds(int p_index, int p_dirIndex)
{
    uint8_t l_dirMask = g_dirMasks.at(p_dirIndex);
    int l_rem = p_index % g_SEGMENTS;
    // Can't go west
    if (l_rem == 0 && l_dirMask & DIR_WEST)
        return false;
    // Can't go east
    if ((l_rem == g_SEGMENTS - 1) && l_dirMask & DIR_EAST)
        return false;

    int l_newIndex = p_index + g_touchingSegments.at(p_dirIndex);
    if (l_newIndex >= (g_TOTALSEGMENTS) || l_newIndex < 0)
        return false;

    return true;
}

// Check if the bit field has 2 opposite directions set, if segment is on edge make sure just 1 bit is set
inline bool opposite_directions(int p_index, uint8_t p_directions)
{
    int l_rem = p_index % g_SEGMENTS;
    // Segment is on the edge, west, east, north, south
    if (l_rem == 0 || l_rem == g_SEGMENTS - 1 || p_index > (g_TOTALSEGMENTS - g_SEGMENTS) || p_index < g_SEGMENTS)
    {
        if (p_directions & DIR_ALL)
            return true;
    }
    if ((p_directions & DIR_NORTH) && (p_directions & DIR_SOUTH))
        return true;
    if ((p_directions & DIR_EAST) && (p_directions & DIR_WEST))
        return true;
    
    return false;
}

// Return the direction that the p_viewing is from the center, this works on the assumption that all the segments are stored in a perfect square
uint8_t direction_from_segment(int p_center, int p_viewing)
{
    if (p_center == p_viewing)
        return 0;

    uint8_t l_dir = 0;
    int l_rowDiff = p_viewing & g_SEGMENTS;
    // Check north or south
    if (p_viewing + (g_SEGMENTS - l_rowDiff) > p_center)
        l_dir = l_dir | DIR_NORTH;
    // Must no just be a lower value but a lower row
    else if (p_viewing - l_rowDiff < p_center)
        l_dir = l_dir | DIR_SOUTH;

    // Check east or west
    int l_centerPosOnRow = p_center % g_SEGMENTS;
    int l_viewingPosOnRow = p_viewing % g_SEGMENTS; 
    if (l_centerPosOnRow < l_viewingPosOnRow)
        l_dir = l_dir | DIR_EAST;
    else if (l_centerPosOnRow > l_viewingPosOnRow)
        l_dir = l_dir | DIR_WEST;

    return l_dir;
}

// Invert the directions of if a bit field is north east invert it to south west Example 0b00000011 to 0b00001100 
inline uint8_t invert_directions(uint8_t p_directions)
{
    uint8_t l_output = 0;
    if (p_directions & DIR_NORTH)
        l_output = l_output | DIR_SOUTH;
    if (p_directions & DIR_SOUTH)
        l_output = l_output | DIR_NORTH;
    if (p_directions & DIR_EAST)
        l_output = l_output | DIR_WEST;
    if (p_directions & DIR_WEST)
        l_output = l_output | DIR_EAST;

    return l_output;
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

        o_segments.push_back(std::move(l_verticesInSeg));
        l_bound += l_dist;
    }
    // Add the last vertices in case of a rounding error
    std::vector<Vertex> l_last;
    while (l_segIndex < p_lats.size())
        l_last.push_back(p_lats.at(l_segIndex++));
    o_segments.push_back(std::move(l_last));
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
        o_segments.push_back(std::move(l_verticesInSeg));
        l_bound += l_dist;
    }
    // Add the last vertices in case of a rounding error
    std::vector<Vertex> l_last;
    while (l_segIndex < p_lngs.size())
        l_last.push_back(p_lngs.at(l_segIndex++));
    o_segments.push_back(std::move(l_last));

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
                if (binary_search_latitude(l_row, l_v) != -1)
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
    // Order of indices is north, east, south, west 
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
            o_n.emplace_back(p_current, p_segment.at(l_indices[i]));
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

// Return a pair of vertices that can be used to connect segments p_dest contains the destination index in the grid and its directional relative to the source
std::pair<Vertex, Vertex> vertices_to_connect_segments(const vectorVertex2d& p_grid, int p_src, const std::pair<size_t, uint8_t>& p_dest, const std::vector<Segment>& p_segmentInfo)
{
    const Segment& l_srcInfo = p_segmentInfo.at(p_src);
    const Segment& l_destInfo = p_segmentInfo.at(p_dest.first);
    // The potential vertices for joining the segments
    std::vector<Vertex> l_srcSegmentJoin;
    std::vector<Vertex> l_destSegmentJoin;
    if (p_dest.second & DIR_NORTH)
    {
        l_srcSegmentJoin.push_back(p_grid.at(p_src).at(l_srcInfo.m_northIndex));
        l_destSegmentJoin.push_back(p_grid.at(p_dest.first).at(l_destInfo.m_southIndex));
    }
    else
    {
        l_srcSegmentJoin.push_back(p_grid.at(p_src).at(l_srcInfo.m_southIndex));
        l_destSegmentJoin.push_back(p_grid.at(p_dest.first).at(l_destInfo.m_northIndex));
    }

    if (p_dest.second & DIR_EAST)
    {
        l_srcSegmentJoin.push_back(p_grid.at(p_src).at(l_srcInfo.m_eastIndex));
        l_destSegmentJoin.push_back(p_grid.at(p_dest.first).at(l_destInfo.m_westIndex));
    }
    else
    {
        l_srcSegmentJoin.push_back(p_grid.at(p_src).at(l_srcInfo.m_westIndex));
        l_destSegmentJoin.push_back(p_grid.at(p_dest.first).at(l_destInfo.m_eastIndex));
    }

    // Find the closest pairing of vertices
    std::pair<int, int> l_closestPair;
    int l_closestDist = INT_MAX;
    for (size_t i = 0; i < l_srcSegmentJoin.size(); i++)
    {
        for (size_t j = 0; j < l_destSegmentJoin.size(); j++)
        {
            double l_x = abs(l_srcSegmentJoin.at(i).get_lng() - l_destSegmentJoin.at(j).get_lng());
            double l_y = abs(l_srcSegmentJoin.at(i).get_lat() - l_destSegmentJoin.at(j).get_lat());
            double l_distance = (l_x * l_x) + (l_y * l_y);
            if (sqrt(l_distance < l_closestDist))
            {
                l_closestDist = l_distance;
                l_closestPair.first = i;
                l_closestPair.second = j;
            }
        }
    }
    // First vertex in the pair is src second is the destination
    return {l_srcSegmentJoin.at(l_closestPair.first), l_destSegmentJoin.at(l_closestPair.second)};
}

void connect_grid(adjacencyList& p_adj, const vectorVertex2d& p_grid, const std::vector<Segment>& p_segmentInfo)
{
    // Connect a Segments together, a segment unless it is on an edge is surround by 8 other segments which can be divided into
    // 4 groups, top 3 north, botton 3 south, right 3 east and left 3 west. There is some overlay in groups.
    // The idea is to connect the segment to all segments until it is connect on 2 opposite sides or reaches and edge.
    // Use bfs to find the nearest 2 segments that contain vertices in opposite directions

    // Keep track of connections made between segments so that they don't have to be made twice
    uint8_t l_segmentDirections[g_TOTALSEGMENTS];
    for (int i = 0; i < g_TOTALSEGMENTS; i++)
        l_segmentDirections[i] = 0;

    for (size_t l_currentSegment = 0; l_currentSegment < p_grid.size(); l_currentSegment++)
    {
        // If the currentSegment is empty skip it
        if (p_grid.at(l_currentSegment).empty())
            continue;

        if (l_currentSegment == 2485)
            std::cout << "We are here!" << std::endl;

        std::unordered_set<int> l_visited;
        l_visited.insert(l_currentSegment);
        int l_level = 1;
        std::vector<size_t> l_frontier {l_currentSegment};
        uint8_t l_neighbourDirections = l_segmentDirections[l_currentSegment];
        // The first value in the pair is the segments index in the grid and the second is its direction in relation to the current segment
        std::vector<std::pair<size_t, uint8_t>> l_neighbours;
        while (!opposite_directions(l_currentSegment, l_neighbourDirections) && !l_frontier.empty())
        {
            std::vector<size_t> l_next;
            for (int l_viewing: l_frontier)
            {
                for (int i = 0; i < DIRECTIONS; i++)
                {
                    size_t l_touching = l_viewing + g_touchingSegments[i];
                    if (segment_in_bounds(l_viewing, i) && l_visited.find(l_touching) == l_visited.end())
                    {
                        l_visited.insert(l_touching);
                        l_next.push_back(l_touching);
                        // Does the segment being touched have any vertices 
                        if (!p_grid.at(l_touching).empty())
                        {
                            uint8_t l_touchingDirection = direction_from_segment(l_currentSegment, l_touching);
                            // If it is the first layer always add it otherwise only add it if the segment is in a new direction
                            if (l_level == 1 || (l_neighbourDirections & l_touchingDirection) == 0)
                            {
                                l_neighbourDirections = l_neighbourDirections | l_touchingDirection;
                                l_neighbours.push_back({l_touching, l_touchingDirection});
                            }
                        }   
                    }
                }
            }
            l_frontier = std::move(l_next);
            l_level++;
        }
        // Connect segments in both directions
        for (const auto l_n: l_neighbours)
        {
            const std::pair<Vertex, Vertex> l_couple = vertices_to_connect_segments(p_grid, l_currentSegment, l_n, p_segmentInfo);
            p_adj[l_couple.first].push_back({l_couple.first, l_couple.second});
            p_adj[l_couple.second].push_back({l_couple.second, l_couple.first});
            l_segmentDirections[l_n.first] = invert_directions(l_n.second);
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
    std::vector<Segment> l_segmentInfo(g_TOTALSEGMENTS);
    create_grid_segments(l_rows, l_cols, l_grid, l_segmentInfo);

    connect_vertices(p_adj, l_grid);
    connect_grid(p_adj, l_grid, l_segmentInfo);
}

// Get the bound of the area parsed, a file must be parsed before and called to this function, otherwise all members will be 0
Bounds& get_bounds()
{
    return g_geoBounds;
}
