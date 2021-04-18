#ifndef _COMMON_H
#define _COMMON_H

#include "vertex.h"
#include "edge.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <SDL2/SDL.h>

typedef std::unordered_map<Vertex, std::vector<Edge>, VertexHash, VertexCompare> adjacencyList;
typedef std::unordered_map<Vertex, Vertex, VertexHash, VertexCompare> previousVertexMap;

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
    std::unordered_set<Vertex, VertexHash, VertexCompare> m_vertices;
    std::unordered_set<Edge, EdgeHash, EdgeCompare> m_edges;
};

#endif