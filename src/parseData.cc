#include "parseData.h"
#include "vertex.h"
#include "edge.h"
#include "common.h"

#include <unordered_map>
#include <vector>
#include <fstream>
#include <algorithm>

#include <cmath>

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

        int l_vlat = abs(stoi(l_lat + l_latd));
        int l_vlng = abs(stoi(l_lng + l_lngd));
        Vertex l_v(l_vlat, l_vlng, l_name);
        l_latvec.emplace_back(l_v);
        l_lngvec.emplace_back(l_v);
        // Move to next line
        std::getline(l_file, l_name);
    }

    std::sort(l_latvec.begin(), l_latvec.end(), [](const Vertex& lhs, const Vertex& rhs){return lhs.m_lat < rhs.m_lat;});
    std::sort(l_lngvec.begin(), l_lngvec.end(), [](const Vertex& lhs, const Vertex& rhs){return lhs.m_lng < rhs.m_lng;});

    // Update the bounds of the area to be mapped
    g_geoBounds.m_west = l_lngvec.at(0).m_lng;
    g_geoBounds.m_east = l_lngvec.at(l_lngvec.size() - 1).m_lng;
    g_geoBounds.m_south = l_latvec.at(0).m_lat;  
    g_geoBounds.m_north = l_latvec.at(l_latvec.size() - 1).m_lat;

    adjacencyList l_adj;
    create_adjacency_list(l_adj, l_latvec, l_lngvec);
    return l_adj;
}

// Search vector for Vertex using latitude member, return -1 if not found
int binary_search_latitude(const std::vector<Vertex>& p_v, double p_item)
{
    int l_low = 0, l_high = p_v.size() - 1;
    while (l_low <= l_high)
    {
        int l_mid = (l_low + l_high) / 2;
        if (p_v.at(l_mid).m_lat == p_item)
            return l_mid;
        else if (p_v.at(l_mid).m_lat < p_item)
            l_low = l_mid + 1;
        else
            l_high = l_mid - 1;

    }
    // Not found 
    return -1;
}

// For each Vertex (City) have a most 4 edges, reaching to the closest in North, East, South and West directions
void create_adjacency_list(adjacencyList& p_adj, const std::vector<Vertex>& p_lats, const std::vector<Vertex>& p_lngs)
{
    for (size_t i = 1; i < p_lats.size() - 1; i++)
    {
        // West and East
        if (i > 0)
            p_adj[p_lats.at(i)].emplace_back(p_lats.at(i), p_lats.at(i - 1));
        if (i < p_lats.size() - 1)
            p_adj.at(p_lats.at(i)).emplace_back(p_lats.at(i), p_lats.at(i + 1));
        // South and North
        int l_lngIndex = binary_search_latitude(p_lngs, p_lats.at(i).m_lat);
        if (l_lngIndex != -1)
        {
            if (l_lngIndex < 1)
                p_adj.at(p_lats.at(i)).emplace_back(p_lats.at(i), p_lngs.at(l_lngIndex - 1));
            if (l_lngIndex > int(p_lngs.size()) - 1)
                p_adj.at(p_lats.at(i)).emplace_back(p_lats.at(i), p_lngs.at(l_lngIndex + 1));
        }
    }    
}

// Get the bound of the area parsed, a file must be parsed before and called to this function, otherwise all member will be 0
Bounds get_bounds()
{
    return g_geoBounds;
}
