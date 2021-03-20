#ifndef _COMMON_H
#define _COMMON_H

#include "vertex.h"
#include "edge.h"

#include <unordered_map>
#include <vector>

typedef std::unordered_map<Vertex, std::vector<Edge>, VertexHash, VertexCompare> adjacencyList;

#endif