#ifndef _COMMON_H
#define _COMMON_H

#include "vertex.h"
#include "edge.h"

#include <unordered_map>
#include <vector>
#include <SDL2/SDL.h>

typedef std::unordered_map<Vertex, std::vector<Edge>, VertexHash, VertexCompare> adjacencyList;

const int WIDTH = 1024;
const int HEIGHT = 1024;

extern SDL_Window* g_window;
extern SDL_Renderer* g_renderer;

#endif