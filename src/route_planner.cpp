#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x /= 100;
    start_y /= 100;
    end_x /= 100;
    end_y /= 100;

    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &model.FindClosestNode(start_x, start_y);
    this->end_node = &model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  
  // https://en.wikipedia.org/wiki/A*_search_algorithm
  // populate current_node.neighbors vector with all the neighbors
  current_node->FindNeighbors();
  float g_value_old = current_node->g_value;   
  
  for(auto node: current_node->neighbors) {
    // For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
    node->parent = current_node;
    node->h_value = CalculateHValue(node);
    node->g_value = g_value_old + node->distance(*current_node);
    
    // add the neighbor to open_list and set the node's visited attribute to true
    open_list.emplace_back(node);
    node->visited = true;
  }
}


// Sort the open_list according to the sum of the h value and g value.
bool Compare(const RouteModel::Node* node_a, const RouteModel::Node* node_b) {
   
  float f_a = node_a->g_value + node_a->h_value;
  float f_b = node_b->g_value + node_b->h_value;
  return f_a > f_b;
}

// sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
  // sort(v->begin(), v->end(), Compare);
  // Create a pointer to the node in the list with the lowest sum.
  sort(this->open_list.begin(), this->open_list.end(), Compare);
  RouteModel::Node* next =  open_list.back();
  // Remove that node from the open_list
  open_list.pop_back();
  // Return the pointer
  return next;
}


// Return the final path found from your A* search.
// This method takes the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // For each node in the chain, add the distance from the node to its parent to the distance variable.
    while(current_node->parent != nullptr) {
      path_found.emplace_back(*current_node);
      // distance += (*current_node).distance(*(*current_node).parent);
      distance += current_node->distance(*current_node->parent);
      current_node = current_node->parent;
    }
  
    path_found.emplace_back(*current_node); // not added in the loop 
  
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    // the start node should be the first element
    //   of the vector, the end node should be the last element.
    // https://en.cppreference.com/w/cpp/algorithm/reverse
    std::reverse(std::begin(path_found), std::end(path_found));  // reverse path
    return path_found;

}


// A* Search algorithm
void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;

  this->open_list.emplace_back(this->start_node);
  this->start_node->visited = true;
  
  while(open_list.size() > 0) {
    // sort the open_list and return the next node
    current_node = this->NextNode();
    
    // When the search has reached the end_node, return the final path that was found.
    if(current_node->x == this->end_node->x && current_node->y == this->end_node->y) {
      // Store the final path in the m_Model.path attribute
      m_Model.path = ConstructFinalPath(end_node);
      break;
    }

    // add all of the neighbors of the current node to the open_list
    AddNeighbors(current_node);
  }
  
  
}