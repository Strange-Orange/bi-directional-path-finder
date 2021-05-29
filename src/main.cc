#include "vertex.h"
#include "edge.h"
#include "common.h"
#include "indexedPriorityQueue.h"
#include "parseData.h"

#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <SDL2/SDL.h>

#include <limits.h>
#include <ctime>

SDL_Window* g_window = nullptr;
SDL_Renderer* g_renderer = nullptr;

std::mutex g_mtx;

bool init();
void quit();
inline void lat_lng_position(int p_lat, int p_lng, const Bounds& p_bounds, int& o_x, int& o_y);
StartEnd place_name_to_vertex(const std::string& p_start, const std::string& p_end, const adjacencyList& p_adj);
void get_edges_in_route(RouteData& p_rd, const Vertex& p_start, const Vertex& p_end, const previousVertexMap& p_prevMap, const adjacencyList& p_adj);
void construct_shortest_path(RouteData& p_rd, const Vertex& p_start, const Vertex& p_end, const SearchData* const ptr_sd);
void search(const adjacencyList& p_adj, const Vertex& p_start, vertexSet& p_visitedVertices, std::atomic<bool>& p_met, Vertex& p_meetingPoint, SearchData* ptr_sd, std::vector<Vertex>& o_lastFrontier);
RouteData bi_directional_dijkstra(const adjacencyList& p_adj, const Vertex& p_start, const Vertex& p_end);
RouteData dijkstra(const adjacencyList& p_adj, const Vertex& p_start, const Vertex& p_end);

// Initialize SDL and create window and renderer
bool init()
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        std::cerr << "Failed to initialize SDL2 : " << SDL_GetError() << std::endl;
        return false; 
    }

    // Set texture filtering to linear
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
    g_window = SDL_CreateWindow("bi-directional path finder", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_SHOWN);
    if (g_window == nullptr)
    {
        std::cerr << "Failed to create SDL2 window: " << SDL_GetError() << std::endl;
        return false;
    }
    g_renderer = SDL_CreateRenderer(g_window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (g_renderer == nullptr)
    {
        std::cerr << "Failed to create SDL2 renderer: " << SDL_GetError() << std::endl;
        return false;
    }
    return true;
}

void quit()
{
    if (g_renderer != nullptr)
    {
        SDL_DestroyRenderer(g_renderer);
        g_renderer = nullptr;
    }
    if (g_window != nullptr)
    {
        SDL_DestroyWindow(g_window);
        g_window = nullptr;
    }

    SDL_Quit();
}

// Convert latitude and longitude to a x and y coordinate
inline void lat_lng_position(int p_lat, int p_lng, const Bounds& p_bounds, int& o_x, int& o_y)
{
    int l_xrange = p_bounds.m_east - p_bounds.m_west;
    int l_yrange = p_bounds.m_north - p_bounds.m_south;
    o_x = ((p_lng - p_bounds.m_west) * (HEIGHT - 5)) / l_xrange; 
    o_y = ((p_lat - p_bounds.m_south) * (WIDTH - 5)) / l_yrange;
    // Flip the y axis otherwise the graph will be upside down
    o_y = (HEIGHT - 5) - o_y;
}

StartEnd place_name_to_vertex(const std::string& p_start, const std::string& p_end, const adjacencyList& p_adj)
{
    StartEnd l_se;
    bool l_startFound = false, l_endFound = false;
    for (const auto& l_item: p_adj)
    {
        if (l_item.first.get_name_c() == p_start)
        {
            l_se.m_start = l_item.first;
            l_startFound = true;
        }
        else if (l_item.first.get_name_c() == p_end)
        {
            l_se.m_end = l_item.first;
            l_endFound = true;
        }
        else
            continue;

        if (l_startFound && l_endFound)
            break;
    }

    return l_se;
}

// Collect all the edges used in the route to the end point
void get_edges_in_route(RouteData& p_rd, const Vertex& p_start, const Vertex& p_end, const previousVertexMap& p_prevMap, const adjacencyList& p_adj)
{
    Vertex l_current = p_end;
    while (l_current != p_start)
    {
        const Vertex& l_prev = p_prevMap.at(l_current);
        for (const auto& l_edge: p_adj.at(l_prev))
        {
            // Found the edge on the route
            if (l_edge.m_dest == l_current.get_name_c())
            {
                p_rd.m_edges.insert(l_edge);
                break;
            }
        }
        l_current = l_prev;
    }
}

// This function is only used when performing bi-direction search
void construct_shortest_path(RouteData& p_rd, const Vertex& p_start, const Vertex& p_end, const SearchData* const ptr_sd)
{
    if (ptr_sd->m_visited.find(p_end) != ptr_sd->m_visited.end())
    {
        Vertex l_current = p_end;
        while (l_current != p_start)
        {
            p_rd.m_vertices.insert(l_current);
            l_current = ptr_sd->m_prev.at(l_current);
        }
    }
}

void search(const adjacencyList& p_adj, const Vertex& p_start, vertexSet& p_visitedVertices, std::atomic<bool>& p_met, Vertex& p_meetingPoint, SearchData* ptr_sd, std::vector<Vertex>& o_lastFrontier)
{
    // Initialize visited, distance and previous maps
    ptr_sd = new SearchData(p_adj);
    Vertex l_meetingPoint;
    

    ptr_sd->m_dist.at(p_start) = 0;
    IndexedPriorityQueue<Vertex, VertexHash> l_pq;
    l_pq.insert(p_start);

    while (!p_met && !l_pq.empty())
    {
        Vertex l_current = l_pq.extract_min();
        ptr_sd->m_visited.at(l_current) = true;
        // Check if a vertex has been visted by the other search, if so break out of the search then set p_met = true
        g_mtx.lock();
        if (p_visitedVertices.find(l_current) != p_visitedVertices.end())
        {
            l_meetingPoint = std::move(l_current);
            break;
        }
        else
            p_visitedVertices.insert(l_current);
        g_mtx.unlock();
        // View all connected vertices
        for (const auto& l_edge: p_adj.at(l_current))
        {
            Vertex l_viewing(l_edge.m_destLat, l_edge.m_destLng, l_edge.m_dest);
            // Ignore already visited vertices
            if (!ptr_sd->m_visited.at(l_viewing))
            {
                int l_newDist = ptr_sd->m_dist.at(l_current) + l_edge.m_cost;
                // Relax the edge
                if (l_newDist < ptr_sd->m_dist.at(l_viewing))
                {
                    ptr_sd->m_prev.at(l_viewing) = l_current;
                    ptr_sd->m_dist.at(l_viewing) = l_newDist;
                    // Update the priority queue
                    Vertex l_updated(l_viewing);
                    l_pq.insert(l_viewing);
                    l_updated.set_cost(l_newDist);
                    l_pq.change_priority(l_viewing, l_updated);
                }
            }
        }
    }
    p_met = true;
    o_lastFrontier = std::move(l_pq.data());
    g_mtx.lock();
    if (p_meetingPoint.get_name_c() == "***")
        p_meetingPoint = std::move(l_meetingPoint);
    g_mtx.unlock();
}

RouteData bi_directional_dijkstra(const adjacencyList& p_adj, const Vertex& p_start, const Vertex& p_end)
{
    vertexSet l_visitedVertices;
    std::atomic<bool> l_met(false);
    std::vector<Vertex> l_forwardLastFrontier;
    std::vector<Vertex> l_backwardLastFrontier;
    previousVertexMap l_forwardSearchPrev;
    previousVertexMap l_backwardSearchPrev;

    Vertex l_meetingPoint;
    // Remember to FREE
    SearchData* l_forwardSearchData = nullptr;
    SearchData* l_backwardSearchData = nullptr;

    std::thread l_backwardSearch(search, std::cref(p_adj), std::cref(p_end), std::ref(l_visitedVertices), std::ref(l_met), std::ref(l_meetingPoint), l_backwardSearchData, std::ref(l_backwardLastFrontier));
    search(p_adj, p_start, l_visitedVertices, l_met, l_meetingPoint, l_forwardSearchData, l_forwardLastFrontier);
    l_backwardSearch.join();

    const Vertex l_originalMeetingPoint = l_meetingPoint;
    int l_smallestDist = l_forwardSearchData->m_dist.at(l_meetingPoint) + l_backwardSearchData->m_dist.at(l_meetingPoint);
    for (const Vertex& l_v: l_forwardLastFrontier)
    {
        if (l_v != l_originalMeetingPoint)
        {
            int l_currentDist = l_forwardSearchData->m_dist.at(l_v);
            // If any of the vertices that l_v are connected to are in the backward frontier and have a shorter distance update the meeting point
            for (const Edge& l_e: p_adj.at(l_v))
            {
                if (l_currentDist + l_e.m_cost < l_smallestDist)
                {
                    l_smallestDist = l_currentDist + l_e.m_cost;
                    l_meetingPoint = Vertex(l_e.m_destLat, l_e.m_destLng, l_e.m_dest);
                    l_forwardSearchData->m_dist.at(l_meetingPoint) = l_smallestDist;
                }
            }
        }
    }

    // Construct the route
    RouteData l_rd;
    std::vector<Vertex> l_forwardRoute;
    std::vector<Vertex> l_backwardRoute;
    construct_shortest_path(l_rd, p_start, l_meetingPoint, l_forwardSearchData);
    construct_shortest_path(l_rd, l_meetingPoint, p_end, l_backwardSearchData);
    get_edges_in_route(l_rd, p_start, l_meetingPoint, l_forwardSearchData->m_prev, p_adj);
    get_edges_in_route(l_rd, l_meetingPoint, p_end, l_backwardSearchData->m_prev, p_adj);

    delete l_forwardSearchData;
    delete l_backwardSearchData;

    return l_rd;

}

RouteData dijkstra(const adjacencyList& p_adj, const Vertex& p_start, const Vertex& p_end)
{
    SearchData l_sd(p_adj);

    l_sd.m_dist.at(p_start) = 0;
    IndexedPriorityQueue<Vertex, VertexHash> l_pq;
    l_pq.insert(p_start);

    while (l_pq.size() != 0 && !l_sd.m_visited.at(p_end))
    {
        Vertex l_current = l_pq.extract_min();
        l_sd.m_visited.at(l_current) = true;
        // View all the connected vertices
        for (const auto& l_edge: p_adj.at(l_current))
        {
            Vertex l_viewing(l_edge.m_destLat, l_edge.m_destLng, l_edge.m_dest);
            // If the viewing vertex has already been visited just ignore it
            if (!l_sd.m_visited.at(l_viewing))
            {
                // std::cout << "viewing : " << l_viewing.get_name_c() << ": Distance: ";
                int l_newDist = l_sd.m_dist.at(l_current) + l_edge.m_cost;
                // Relax the edge
                if (l_newDist < l_sd.m_dist.at(l_viewing))
                {
                    // std::cout << l_newDist << "\n";
                    l_sd.m_prev.at(l_viewing) = l_current;
                    // Edge to the viewing vertex
                    l_sd.m_dist.at(l_viewing) = l_newDist;
                    // Update the priority queue
                    Vertex l_updated(l_viewing);
                    l_pq.insert(l_updated);
                    l_updated.set_cost(l_newDist);
                    l_pq.change_priority(l_viewing, l_updated);
                }
            }
        }
    }

    RouteData l_rd;
    if (l_sd.m_visited.find(p_end) != l_sd.m_visited.end())
    {
        Vertex l_viewing = p_end;
        while (l_viewing.get_name_c() != p_start.get_name_c())
        {
            l_rd.m_vertices.insert(l_viewing);
            l_viewing = l_sd.m_prev.at(l_viewing);
        }
        get_edges_in_route(l_rd, p_start, p_end, l_sd.m_prev, p_adj);
        return l_rd;
    }
    return l_rd;
}

int main(int argc, char* args[])
{
    if (!init())
        return 1;

    adjacencyList l_adj;
    if (argc == 4)
        l_adj = parse_data_csv(args[1]);

    // Look for a default in the current directory named data.csv
    else
    {
        std::cout << "Enter a csv file, start point and end point\n";
        return 1;        
    }

    StartEnd l_se = place_name_to_vertex(args[2], args[3], l_adj);
    // If the start point for the end point are not in the adjacency list return 1
    if (l_se.m_start.get_name_c() == "***" || l_se.m_end.get_name_c() == "***")
    {
        std::cout << "Start or end point are not in the csv file\n";
        return 1;
    }
    else
    {
        l_se.m_start.set_cost(0);
        l_se.m_end.set_cost(0);
    }
    
    std::clock_t l_start;
    l_start = std::clock();
    RouteData l_route = dijkstra(l_adj, l_se.m_start, l_se.m_end);
    double l_duration = (std::clock() - l_start) / (double)CLOCKS_PER_SEC;
    std::cout << l_duration << "\n";
    // Print all locations visited on the route
    for (const Vertex& l_cv: l_route.m_vertices)
        std::cout << l_cv.get_name() << " : ";
    std::cout << std::endl;

    std::unordered_set<Edge, EdgeHash, EdgeCompare> l_routeEdges;

    std::vector<SDL_Rect> l_cityRects(l_adj.size() - l_route.m_vertices.size());
    std::vector<SDL_Rect> l_routeRects(l_route.m_vertices.size());
    Bounds geoBounds = get_bounds();

    bool l_quit = false;
    SDL_Event l_event;

    // Main loop
    while (!l_quit)
    {
        while (SDL_PollEvent(&l_event) != 0)
        {
            if (l_event.type == SDL_QUIT)
            {
                l_quit = true;
            }
            if (l_event.type == SDL_KEYDOWN)
            {
                if (l_event.key.keysym.sym == SDLK_ESCAPE)
                {
                    l_quit = true;
                }
            }
        }
        // Render loop

        SDL_SetRenderDrawColor(g_renderer, 0, 0, 0, 255);
        SDL_RenderClear(g_renderer);

        SDL_SetRenderDrawColor(g_renderer, 0, 0, 255, 255);
        std::unordered_set<Edge, EdgeHash, EdgeCompare> l_edges;
        int l_vIndex = 0;
        int l_vRouteIndex = 0;
        for (const std::pair<Vertex, std::vector<Edge>>& l_item: l_adj)
        {
            int l_x, l_y;
            lat_lng_position(l_item.first.get_lat(), l_item.first.get_lng(), geoBounds, l_x, l_y);
            if (l_route.m_vertices.find(l_item.first) == l_route.m_vertices.end())
                l_cityRects.at(l_vIndex++) = {l_x, l_y, 5, 5};
            else
                l_routeRects.at(l_vRouteIndex++) = {l_x, l_y, 5, 5};

            for (const Edge& l_e: l_item.second)
            {
                if (l_edges.find(l_e) == l_edges.end())
                {
                    l_edges.insert(l_e);
                    int l_dx, l_dy;
                    lat_lng_position(l_e.m_destLat, l_e.m_destLng, geoBounds, l_dx, l_dy);
                    // Change draw colour if the edge is part of the route
                    if (l_route.m_edges.find(l_e) != l_route.m_edges.end())
                        SDL_SetRenderDrawColor(g_renderer, 255, 0, 0, 255);
                    else
                        SDL_SetRenderDrawColor(g_renderer, 0, 0, 255, 255);
                    SDL_RenderDrawLine(g_renderer, l_x, l_y, l_dx, l_dy);
                }
            }
        }
        SDL_RenderFillRects(g_renderer, l_cityRects.data(), l_cityRects.size());
        SDL_SetRenderDrawColor(g_renderer, 255, 0, 0, 255);
        SDL_RenderFillRects(g_renderer, l_routeRects.data(), l_routeRects.size());
        
        SDL_RenderPresent(g_renderer);
    } 

    quit();
    return 0;
}
