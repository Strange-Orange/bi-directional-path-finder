#ifndef _COMMON_H
#define _COMMON_H

#include "vertex.h"
#include "edge.h"

#include <array>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <SDL2/SDL.h>

#include <limits.h>

typedef std::unordered_map<Vertex, std::vector<Edge>, VertexHash, VertexCompare> adjacencyList;
typedef std::unordered_map<Vertex, Vertex, VertexHash, VertexCompare> previousVertexMap;
typedef std::unordered_set<Vertex, VertexHash, VertexCompare> vertexSet;

const int WIDTH = 1024;
const int HEIGHT = 1024;

extern SDL_Window* g_window;
extern SDL_Renderer* g_renderer;

struct StartEnd
{
    Vertex m_start;
    Vertex m_end;
};

struct RouteData
{
    std::vector<Vertex> m_route;
    std::unordered_set<Vertex, VertexHash, VertexCompare> m_vertices;
    std::unordered_set<Edge, EdgeHash, EdgeCompare> m_edges;
};

struct SearchData
{
    std::unordered_map<Vertex, bool, VertexHash, VertexCompare> m_visited;
    std::unordered_map<Vertex, int, VertexHash, VertexCompare> m_dist;
    previousVertexMap m_prev;

    SearchData() = default;
    SearchData(const adjacencyList& p_adj)
    {
        for (const auto& l_item: p_adj)
        {
            m_visited[l_item.first] = false;
            m_dist[l_item.first] = INT_MAX;
            m_prev[l_item.first];
        }
    }
};

#endif