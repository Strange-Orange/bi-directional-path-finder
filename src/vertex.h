#ifndef _VERTEX_H
#define _VERTEX_H

#include <string>
#include <functional>


struct Vertex
{
    double m_lat;
    double m_lng;
    std::string m_name;

    Vertex()
        : m_lat(0), m_lng(0), m_name("***") {}

    Vertex(double p_lat, double p_lng, std::string p_name)
        : m_lat(p_lat), m_lng(p_lng), m_name(p_name) {};

    Vertex(const Vertex& p_v)
        : m_lat(p_v.m_lat), m_lng(p_v.m_lng), m_name(p_v.m_name) {};

    Vertex(Vertex&& p_v)
        : m_lat(p_v.m_lat), m_lng(p_v.m_lng), m_name(p_v.m_name) {};

    Vertex& operator=(const Vertex& rhs)
    {
        this->m_lat = rhs.m_lat;
        this->m_lng = rhs.m_lng;
        this->m_name = rhs.m_name;
        return *this;
    }

    bool operator==(const Vertex& rhs)
    {
        return (this->m_lat == rhs.m_lat) && (this->m_lng == rhs.m_lng) && (this->m_name == rhs.m_name);
    }
};


struct VertexHash
{
    std::hash<std::string> m_stringHash;
    size_t operator()(const Vertex& p_v) const 
    {
        return m_stringHash(p_v.m_name);
    }
};

#endif