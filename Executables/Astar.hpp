#include <bits/stdc++.h>
/*
 * Astar Algorithm function that reads from a text file for different mazes,
 * extracts the edges from the int pairs solves for a path and returns a
 * direction from each junction node (node with neighbours>2)
 * @returns: 0 straight, -1 left, 1 right
 * @params: None
 */
#ifndef __ASTAR__
#define __ASTAR__
std::vector<int> Astar(std::string map);
#endif // !
