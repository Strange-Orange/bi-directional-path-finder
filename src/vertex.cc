#include "vertex.h"

Vertex::Vertex()
    : m_lat(0), m_lng(0), m_name("***") {}

Vertex::Vertex(int p_lat, int p_lng, std::string p_name)
    : m_lat(p_lat), m_lng(p_lng), m_name(p_name), m_cost(INT_MAX) {};

Vertex::Vertex(const Vertex& p_v)
    : m_lat(p_v.m_lat), m_lng(p_v.m_lng), m_name(p_v.m_name), m_cost(p_v.m_cost) {};

Vertex::Vertex(Vertex&& p_v)
    : m_lat(p_v.m_lat), m_lng(p_v.m_lng), m_name(p_v.m_name), m_cost(p_v.m_cost) {};

Vertex& Vertex::operator=(const Vertex& rhs)
{
    this->m_lat = rhs.m_lat;
    this->m_lng = rhs.m_lng;
    this->m_name = rhs.m_name;
    this->m_cost = rhs.m_cost;
    return *this;
}

bool Vertex::operator==(const Vertex& rhs) const
{
    return (this->m_lat == rhs.m_lat) && (this->m_lng == rhs.m_lng) && (this->m_name == rhs.m_name);
}

bool Vertex::operator!=(const Vertex& rhs) const 
{
    return (this->m_lat != rhs.m_lat) && (this->m_lng != rhs.m_lng) && (this->m_name != rhs.m_name);
}

// Operator overloads to be used in the priority queue
bool Vertex::operator>(const Vertex& rhs) const
{
    return this->m_cost > rhs.m_cost;
}

bool Vertex::operator>=(const Vertex& rhs) const
{
    return this->m_cost >= rhs.m_cost;
}

bool Vertex::operator<(const Vertex& rhs) const 
{
    return this->m_cost < rhs.m_cost;
}

int Vertex::get_lat() const
{
    return m_lat;
}

int Vertex::get_lng() const
{
    return m_lng;
}

std::string Vertex::get_name() const 
{
    return m_name;
}

const std::string& Vertex::get_name_c() const
{
    return m_name;
}

int Vertex::get_cost() const
{
    return m_cost;
}

void Vertex::set_cost(int p_cost)
{
    m_cost = p_cost;
}
