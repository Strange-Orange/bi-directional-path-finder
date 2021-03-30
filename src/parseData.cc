#include "parseData.h"
#include "vertex.h"
#include "edge.h"
#include "common.h"

#include <iostream>
#include <unordered_map>
#include <vector>
#include <utility>
#include <fstream>
#include <algorithm>

#include <cmath>

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
        // Move to next line
        std::getline(l_file, l_name);
    }
    std::cout << l_latvec.size() << std::endl;
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
        // Avoid making a copy
        o_segments.push_back(std::move(l_verticesInSeg));
        l_bound += l_dist;
    }
    // Add the last vertices in case of a rounding error
    std::vector<Vertex> l_last;
    while (l_segIndex < p_lats.size())
        l_last.push_back(p_lats.at(l_segIndex++));
    o_segments.push_back(std::move(l_last));
}

// Create 10 latitude segments. Place each vertex in the segment where its latitude fits in the latitude range of the segment
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
        // Avoid making a copy
        o_segments.push_back(std::move(l_verticesInSeg));
        l_bound += l_dist;
    }
    // Add the last vertices in case of a rounding error
    std::vector<Vertex> l_last;
    while (l_segIndex < p_lngs.size())
        l_last.push_back(p_lngs.at(l_segIndex++));
    o_segments.push_back(std::move(l_last));

}

void create_grid_segments(const vectorVertex2d& p_rows, const vectorVertex2d& p_cols, vectorVertex2d& o_grid)
{
    for (const std::vector<Vertex>& l_row: p_rows)
    {
        for (const std::vector<Vertex>& l_col: p_cols)
        {
            std::vector<Vertex> l_currentGridSpace;
            for (const Vertex& l_cVertex: l_col)
            {
                if (binary_search_latitude(l_row, l_cVertex.get_lat()))
                {
                    l_currentGridSpace.push_back(l_cVertex);
                }
            }
            o_grid.push_back(std::move(l_currentGridSpace));   
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
    create_grid_segments(l_rows, l_cols, l_grid);
    
    for (const std::vector<Vertex>& l_segment: l_grid)
    {
        for (const Vertex& l_source: l_segment)
        {
            p_adj[l_source];
            g_geoBounds.m_cities[l_source.get_name_c()] = l_source;
            // for (const Vertex& l_dest: l_segment)
            // {
            //     if (l_source != l_dest)
            //     {
            //         p_adj.at(l_source).push_back({l_source, l_dest});
            //     }
            // }
        }
    }
    // for (size_t i = 0; i < l_grid.size(); i++)
    // {
    //     std::cout << "Section: " << i << " : ";
    //     for (const Vertex& p_v: l_grid.at(i))
    //     {
    //         std::cout << p_v.get_name_c() << ", ";
    //     }
    //     std::cout << std::endl;
    // }
}

// Get the bound of the area parsed, a file must be parsed before and called to this function, otherwise all members will be 0
Bounds& get_bounds()
{
    return g_geoBounds;
}
