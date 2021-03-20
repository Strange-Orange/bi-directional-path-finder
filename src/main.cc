#include "vertex.h"
#include "edge.h"
#include "common.h"
#include "indexedPriorityQueue.h"
#include "parseData.h"

#include <iostream>
#include <string>


int main(int argc, char* args[])
{
    adjacencyList l_adj = parse_data_csv("./data/cn.csv");
    Bounds geoBounds = get_bounds();
    std::cout << geoBounds.m_east << " : " << geoBounds.m_west << " : " << geoBounds.m_south << " : " << geoBounds.m_north << "\n";
    return 0;
}