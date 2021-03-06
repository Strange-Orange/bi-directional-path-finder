# bi-directional-path-finder

Created as an experiment to have a look at how a concurrent version of bi-directional Dijkstra performs.

A graph is created using csv files from [Simple-maps](https://simplemaps.com/), they have a subset of their city data that can be used under a MIT license. A virtual road network is created by simply join cities that are close together. The graph created is then used to find the shortest path between two cities that appear in the graph.

## Dependencies
The only external library in use is [SDL2](https://www.libsdl.org/), which is used for window management and drawing the "road network".

## Usage
The first expected argument is a path to a csv file used for the cities in the graph, the second is the city in which to start from and the final argument is the desired end point. If neither the start or end are not in the graph the search will be terminated. 

## Example and output
```
bin/bi-dir-path-finder.out data/cn.csv Beijing Shenzhen
```
![roadmap](images/example0.png)

Time information for the search is displayed in the terminal

![time-taken](images/example1.png)




