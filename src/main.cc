#include "vertex.h"
#include "edge.h"
#include "common.h"
#include "indexedPriorityQueue.h"
#include "parseData.h"

#include <iostream>
#include <string>
#include <unordered_set>
#include <utility>
#include <SDL2/SDL.h>

SDL_Window* g_window = nullptr;
SDL_Renderer* g_renderer = nullptr;

bool init();
void quit();
inline void lat_lng_position(const Vertex& p_v, const Bounds& p_bounds, int& o_x, int& o_y);

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

// FIX: Does not work for areas that are not in the north east, all locations have position latitude and longitude
inline void lat_lng_position(const Vertex& p_v, const Bounds& p_bounds, int& o_x, int& o_y)
{
    int l_xrange = p_bounds.m_east - p_bounds.m_west;
    int l_yrange = p_bounds.m_north - p_bounds.m_south;
    o_x = ((p_v.get_lng() - p_bounds.m_west) * (HEIGHT - 5)) / l_xrange; 
    o_y = ((p_v.get_lat() - p_bounds.m_south) * (WIDTH - 5)) / l_yrange;
    // Flip the y axis otherwise the graph will be upside down
    o_y = (HEIGHT - 5) - o_y;
}

int main(int argc, char* args[])
{
    if (!init())
        return 1;

    adjacencyList l_adj = parse_data_csv("./data/cn.csv");
    std::vector<SDL_Rect> l_cityRects(l_adj.size());
    Bounds geoBounds = get_bounds();
    std::cout << geoBounds.m_east << " : " << geoBounds.m_west << " : " << geoBounds.m_south << " : " << geoBounds.m_north << "\n";

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
        int l_vIndex = 0;
        for (const std::pair<Vertex, std::vector<Edge>>& l_item: l_adj)
        {
            int l_x, l_y;
            lat_lng_position(l_item.first, geoBounds, l_x, l_y);
            l_cityRects.at(l_vIndex++) = {l_x, l_y, 5, 5};
        }
        SDL_RenderFillRects(g_renderer, l_cityRects.data(), l_cityRects.size());
        
        SDL_RenderPresent(g_renderer);
    } 

    quit();
    return 0;
}
