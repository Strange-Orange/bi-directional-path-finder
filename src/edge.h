#ifndef _EDGE_H
#define _EDGE_H

#include "vertex.h"

#include <string>
#include <functional>

#include <cmath>

struct Edge
{
    // Cost will be distance as the crow flies, model as if it were flat. May change this later
    double m_cost;
    int m_srcLat;
    int m_srcLng;
    int m_destLat;
    int m_destLng;
    std::string m_dest;

    Edge()
        : m_cost(0), m_srcLat(0), m_srcLng(0), m_destLat(0), m_destLng(0), m_dest("***") {}

    Edge(const Vertex& p_start, const Vertex& p_end)
    {
        // Use the latitude and longitude as vectors
        double l_x = abs(p_end.get_lng() - p_start.get_lng());
        double l_y = abs(p_end.get_lat() - p_start.get_lat());
        m_cost = sqrt((l_x * l_x) + (l_y * l_y));
        m_srcLat = p_start.get_lat();
        m_srcLng = p_start.get_lng();
        m_destLat = p_end.get_lat();
        m_destLng = p_end.get_lng();
        m_dest = p_end.get_name();
    }
};

struct EdgeHash
{
    std::hash<int> m_intHash;

    size_t operator()(const Edge& p_e) const 
    {
        return m_intHash(p_e.m_srcLat + p_e.m_srcLng + p_e.m_destLat + p_e.m_destLng);
    }  
};

struct EdgeCompare 
{
    bool operator()(const Edge& p_lhs, const Edge& p_rhs) const
    {
        return (p_lhs.m_srcLat + p_lhs.m_srcLng + p_lhs.m_destLat + p_lhs.m_destLng) == (p_rhs.m_srcLat + p_rhs.m_srcLng + p_rhs.m_destLat + p_rhs.m_destLng);
    }
};

#endif