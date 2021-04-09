#ifndef _VERTEX_H
#define _VERTEX_H

#include <string>
#include <functional>

#include <limits.h>

class Vertex
{
    public:
        Vertex();
        Vertex(int p_lat, int p_lng, std::string p_name);
        Vertex(const Vertex& p_v);
        Vertex(Vertex&& p_v);
        Vertex& operator=(const Vertex& rhs);
        bool operator==(const Vertex& rhs) const;
        bool operator!=(const Vertex& rhs) const;
        bool operator>(const Vertex& rhs) const;
        bool operator>=(const Vertex& rhs) const;
        bool operator<(const Vertex& rhs) const;
        int get_lat() const;
        int get_lng() const;
        std::string get_name() const;
        const std::string& get_name_c() const;
        int get_cost() const;
    private:
        int m_lat;
        int m_lng;
        std::string m_name;
        int m_cost;
};

struct VertexHash
{
    std::hash<std::string> m_stringHash;
    size_t operator()(const Vertex& p_v) const 
    {
        return m_stringHash(p_v.get_name());
    }
};

struct VertexCompare
{
    bool operator()(const Vertex& lhs, const Vertex& rhs) const
    {
        return (lhs.get_lat() == rhs.get_lat()) && (lhs.get_lng() == rhs.get_lng()) && (lhs.get_name() == rhs.get_name());
    }
};

#endif
