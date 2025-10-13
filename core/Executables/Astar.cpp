#include "ament_index_cpp/get_package_share_directory.hpp"
#include <bits/stdc++.h>
#include <cmath>
#include <fstream>  // ifstream
#include <iomanip>  // ws
#include <iostream> // cout, endl
#include <map>      // map
#include <sstream>  // stringstream
using namespace std;

/*
 * Astar Algorithm function that reads from a text file for different mazes,
 * extracts the edges from the int pairs solves for a path and returns a
 * direction from each junction node (node with neighbours>2)
 * @returns:  0 straight, -1 left, 1 right
 * Type: vector<int>
 * @params: None
 */
vector<int> Astar(std::string map) {
  std::string package_path =
      ament_index_cpp::get_package_share_directory("navigation_bot");
  std::string file_path =
      package_path + "/Maze/spanning_tree.txt"; // adjust folder
  std::map<string, string> configuration;
  ifstream fin(file_path);
  string line;
  vector<pair<int, int>> edges;
  while (getline(fin, line)) { // loop through every line in the file
    string key;
    string value;
    stringstream ss(line); // make a stream from the line
    getline(ss, key, ':'); // read key until :
    ss >> ws;              // ignore whitespaces
    getline(ss, value);    // read value until newline

    // Store them
    if (key == map) {
      std::stringstream ss(value);
      char ch;
      int a, b;

      // Parse all pairs (a, b)
      while (ss >> ch) {
        if (ch == '(') {
          ss >> a >> ch >> b; // read `a , b`
          edges.emplace_back(a, b);
        }
      }
    }
  }
  const int N = 100;
  vector<vector<int>> adj(N);
  for (auto [u, v] : edges) {
    adj[u].push_back(v);
    adj[v].push_back(u);
  }
  adj[0].push_back(-1);
  adj[99].push_back(100);

  int start = 0, goal = 99;

  // A* setup
  vector<int> cameFrom(N, -1);
  vector<int> gScore(N, INT_MAX);
  gScore[start] = 0;

  // Manhattan distance heuristic on a 10x10 grid
  auto heuristic = [&](int a, int b) {
    int ax = a % 10, ay = a / 10;
    int bx = b % 10, by = b / 10;
    return abs(ax - bx) + abs(ay - by);
  };

  auto fScore = [&](int node) { return gScore[node] + heuristic(node, goal); };

  using P = pair<int, int>; // (fScore, node)
  priority_queue<P, vector<P>, greater<P>> openSet;
  openSet.push({fScore(start), start});

  while (!openSet.empty()) {
    int current = openSet.top().second;
    openSet.pop();

    if (current == goal)
      break; // reached the goal

    for (int neighbor : adj[current]) {
      int tentative_g = gScore[current] + 1; // all edges weight = 1
      if (tentative_g < gScore[neighbor]) {
        cameFrom[neighbor] = current;
        gScore[neighbor] = tentative_g;
        openSet.push({fScore(neighbor), neighbor});
      }
    }
  }

  // Reconstruct path
  vector<int> path;
  for (int cur = goal; cur != -1; cur = cameFrom[cur]) {
    path.push_back(cur);
  }
  reverse(path.begin(), path.end());
  path.insert(path.begin(), -1);
  path.push_back(100);
  cout << "Shortest path from " << -1 << " to " << 100 << ":\n";
  for (int node : path)
    cout << node << " ";
  cout << "\nPath length: " << path.size() - 1 << endl;

  cout << "Direction to move at junctions\n";
  vector<int> direction;
  for (int node = 1; node < path.size() - 1; node++) {
    if (adj[path[node]].size() > 2) {
      int ax = path[node - 1] % 10, ay = path[node - 1] / 10;
      int bx = path[node] % 10, by = path[node] / 10;
      int cx = path[node + 1] % 10, cy = path[node + 1] / 10;
      pair<int, int> vectorbase = {bx - ax, by - ay};
      pair<int, int> vectornext = {cx - ax, cy - ay};
      double signed_angle = atan2(vectornext.second, vectornext.first) -
                            atan2(vectorbase.second, vectorbase.first);
      if (signed_angle > 0)
        direction.push_back(1);
      else if (signed_angle < 0)
        direction.push_back(-1);
      else
        direction.push_back(0);
    }
  }
  return direction;
}

// int main() {
//   vector<int> direction = Astar("12");
//
//   for (auto dir : direction) {
//     cout << dir << "\n";
//   }
// }
