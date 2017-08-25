// Implementation of Dijkstra algorithm to find shortest path between two points
// Sample run:
//   $g++ -std=c++11 -o shortest_path.bin shortest_path.cpp
//   $./shortest_path.bin citymapper-coding-test-graph.dat 316319897 316319936
// first argument is a file with input graph, second and third arguments are
// ids of origin and destination. 
#include <cassert>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <queue>

typedef long long Node;
typedef std::map<Node, std::set<Node>> AdjacencySet;
typedef std::map<std::pair<Node, Node>, double> DistanceMap;

void ReadGraph(const std::string& filename, DistanceMap *distance,
               AdjacencySet *neighbours) {
  std::fstream input(filename.c_str(), std::ios_base::in);
  
  long number_of_nodes;
  Node id;
  input >> number_of_nodes;
  for (long i = 0; i < number_of_nodes; ++i) {
    input >> id;
    (*neighbours)[id] = {};
  }
  
  long number_of_edges;
  input >> number_of_edges;
  Node from, to;
  double length;
  for (long i = 0; i < number_of_edges; ++i) {
    input >> from >> to >> length;
    assert(neighbours->count(from) != 0);
    assert(neighbours->count(to) != 0);
    (*distance)[std::make_pair(to, from)] = length;
    (*distance)[std::make_pair(from, to)] = length;
    (*neighbours)[to].insert(from);
    (*neighbours)[from].insert(to);
  }
}

double ShortestDistance(Node from, Node to,
                        const DistanceMap &distance,
                        const AdjacencySet &neighbours) {
  typedef std::pair<double, Node> NodeDistance;
  std::priority_queue <NodeDistance, std::vector<NodeDistance>,
                       std::greater<NodeDistance>> nodes_in_process;
  nodes_in_process.push(std::make_pair(0.0, from));
  std::set<Node> visited_nodes;
  std::map<Node, double> current_estimate;
  while (!nodes_in_process.empty()) {
    NodeDistance top = nodes_in_process.top();
    nodes_in_process.pop();
    if (visited_nodes.count(top.second) != 0) continue;
    visited_nodes.insert(top.second);
    if (top.second == to) return top.first;

    current_estimate[top.second] = top.first;
    for (Node neighbour : neighbours.at(top.second)) {
      if (visited_nodes.count(neighbour) != 0) continue;
      auto edge = std::make_pair(top.second, neighbour);
      if (distance.count(edge) == 0) continue;
      double new_estimate = top.first + distance.at(edge);
      if (current_estimate.count(neighbour) == 0 ||
          new_estimate < current_estimate[neighbour]) {
        current_estimate[neighbour] = new_estimate;
        nodes_in_process.push(std::make_pair(new_estimate, neighbour));
      }
    }
  }

  return -1;
}

int main(int argc, char* argv[]) {
  assert(argc == 4);

  Node from = std::stoll(argv[2]);
  Node to = std::stoll(argv[3]);

  DistanceMap distance;
  AdjacencySet neighbours;
  ReadGraph(argv[1], &distance, &neighbours);
  assert(neighbours.count(from) != 0);
  assert(neighbours.count(to) != 0);
  
  std::cout << "Shortest distance between " << from << " and " << to << ": "
            << ShortestDistance(from, to, distance, neighbours) << std::endl;
  
  return 0;
}
