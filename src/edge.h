#ifndef _EDGE_H
#define _EDGE_H

#include "vertex.h"

#include <string>

#include <cmath>

struct Edge
{
    // Cost will be distance as the crow flies, model as if it were flat. May change this later
    double m_cost;
    int m_destLat;
    int m_destLng;
    std::string m_dest;

    Edge()
        : m_cost(0), m_dest("***") {}

    Edge(const Vertex& p_start, const Vertex& p_end)
    {
        // Use the latitude and longitude as vectors
        double l_x = abs(p_end.get_lng() - p_start.get_lng());
        double l_y = abs(p_end.get_lat() - p_start.get_lat());
        m_cost = sqrt((l_x * l_x) + (l_y * l_y));
        m_destLat = p_end.get_lat();
        m_destLng = p_end.get_lng();
        m_dest = p_end.get_name();
    }
};

#endif