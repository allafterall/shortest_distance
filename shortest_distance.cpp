// Implementation of Dijkstra algorithm to find shortest path between two points 
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
// For each node we keep set of adjacent nodes.
typedef std::map<Node, std::set<Node>> AdjacencySet;
// For each nodes connected with edge we keep distance between them.
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

// Calculate shortest path distance between nodes from and to given map of
// distances between nodes and set of neighbours for each node.
// Please note that distance must be non-negative.
// Also AdjacencySet must be initialized for each available node id (in case 
// some node has no neighbours set should be empty)
// Returns shortest distance if nodes are connected and -1 otherwise.
double ShortestDistance(Node from, Node to,
                        const DistanceMap &distance,
                        const AdjacencySet &neighbours) {
  assert(neighbours.count(from) != 0);
  assert(neighbours.count(to) != 0);

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
      assert(distance.at(edge) >= 0);
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

bool CheckResult(double actual, double expected) {
  // Normally we should not compare doubles like that but it's fine as long as
  // values are integer.
  std::cout << "\t" << (actual == expected ? "Pass" : "Fail") << ", expected: " 
            << expected << ", actual: " << actual << std::endl;
  return actual == expected;
}

void RunTests() {
  DistanceMap distance;
  AdjacencySet neighbours;
  bool all_pass = true;

  distance = {};
  neighbours = {{0, {}}};
  std::cout << "Only one node:" << std::endl; 
  all_pass &= CheckResult(ShortestDistance(0, 0, distance, neighbours), 0);
  
  distance = {{{0, 1}, 1}, {{1, 0}, 1}};
  neighbours = {{0, {1}}, {1, {0}}};
  std::cout << "Two connected nodes:" << std::endl;
  all_pass &= CheckResult(ShortestDistance(0, 1, distance, neighbours), 1);

  distance = {};
  neighbours = {{0, {}}, {1, {}}};
  std::cout << "Two disconnected nodes:" << std::endl;
  all_pass &= CheckResult(ShortestDistance(0, 1, distance, neighbours), -1);

  distance = {};
  neighbours = {};
  ReadGraph("data/test_graph.txt", &distance, &neighbours);
  std::cout << "Graph with two connectivity components:" << std::endl;
  all_pass &= CheckResult(ShortestDistance(0, 3, distance, neighbours), 6);
  all_pass &= CheckResult(ShortestDistance(2, 1, distance, neighbours), 2);
  all_pass &= CheckResult(ShortestDistance(0, 3, distance, neighbours), 6);
  all_pass &= CheckResult(ShortestDistance(3, 7, distance, neighbours), -1);
  all_pass &= CheckResult(ShortestDistance(4, 5, distance, neighbours), 6);
  all_pass &= CheckResult(ShortestDistance(4, 7, distance, neighbours), 0);    
  
  distance = {{{0, 999}, 1000}, {{999, 0}, 1000},
              {{998, 999}, 1}, {{999, 998}, 1}};
  neighbours = {{0, {999}}, {999, {0, 998}}, {998, {999}}};
  for (int i = 1; i < 999; ++i) {
    distance[std::make_pair(i, i - 1)] = 1;
    distance[std::make_pair(i - 1, i)] = 1;
    neighbours[i].insert(i - 1);
    neighbours[i - 1].insert(i);
  }
  std::cout << "Cirle graph with one edge of 1000 and 999 edges of 1:"
            << std::endl;
  all_pass &= CheckResult(ShortestDistance(999, 0, distance, neighbours), 999);
  all_pass &= CheckResult(ShortestDistance(1, 998, distance, neighbours), 997);

  if (!all_pass) {
    std::cout << "Some tests are failing!" << std::endl;
  } else {
    std::cout << "All tests are passing!" << std::endl;
  }
}

int main(int argc, char* argv[]) {
  if (argc == 1) {
    std::cout << "Running tests" << std::endl;
    RunTests();
    return 0;
  }

  assert(argc == 4);
  
  Node from = std::stoll(argv[2]);
  Node to = std::stoll(argv[3]);

  DistanceMap distance;
  AdjacencySet neighbours;
  ReadGraph(argv[1], &distance, &neighbours);
  
  std::cout << "Shortest distance between " << from << " and " << to << ": "
            << ShortestDistance(from, to, distance, neighbours) << std::endl;
  
  return 0;
}
