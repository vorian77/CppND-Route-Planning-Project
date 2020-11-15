#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// TODO 3: Implement the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // For A* search, the heuristic-value for a node is the distance between the node and the goal
    return node->distance(*end_node);
}

float RoutePlanner::CalculateGValue(RouteModel::Node const *node, RouteModel::Node const *parent_node) {
    // For A* search, the g-value for a node is the sum of the node's parent's g-value,
    // and the distance between the node and its parent
    return parent_node->g_value + node->distance(*parent_node);
}

// TODO 4: Complete the AddNeighbors method 
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
     // set neighbors of current node
    current_node->FindNeighbors();
    
    // for each neighbor... 
    for (auto neighbor : current_node->neighbors) {
        // set parent value, h_value, g_value
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = CalculateGValue(neighbor, current_node);
        
        // add neighbor to open_list
        this->open_list.push_back(neighbor);

        // update neighbor's visited flag
        neighbor->visited = true;
    }
}

// TODO 5: Complete the NextNode method

// The following sort and sort helper functions were derived from the
// Udacity C++ Nanodegree Lesson 2-Foundations A-Star search project
// - GetFValue(), Compare(), Sort() 
float GetFValue(RouteModel::Node *node) {
    // For A* Search, the f-value of a node is the sum of its g-value and its h-value
    float g_value = node->g_value;
    float h_value = node->h_value;
    return g_value + h_value;
}

bool Compare(RouteModel::Node * node1, RouteModel::Node * node2) {
    //returns true if the f-value of node1 is less than the f-value of node2, otherwise false
    return GetFValue(node1) < GetFValue(node2);
}

void CellSort(std::vector<RouteModel::Node*> *nodes) {
    // sort the list of nodes from lowest to highest f-value
    std::sort(nodes->begin(), nodes->end(), Compare);
}

RouteModel::Node *RoutePlanner::NextNode() {
    // return the node in the open list with the lowest f-value

    // sort open_list by f-value, lowest to highest
    CellSort(&open_list);
 
    // - Create a pointer to the lowest f_value node in the list
    RouteModel::Node *smallest_f_value_node = open_list.front();
     
    // - Remove that node from the open_list
    open_list.erase(open_list.begin());
    
    // - Return the pointer.  
    return smallest_f_value_node;  
}

// TODO 6: Complete the ConstructFinalPath method
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here. 
        
     while (current_node->parent) {
        // traverse up the map from the end_node to the start_node (the only node with a null parent)

        // build the path by inserting new nodes at the beginnig of the list,
        // which reverses the order and produces the proper driving direction
        // between the nodes 
        path_found.insert(path_found.begin(), *current_node);

        // increment the total path distance counter with the distance between current node and it's parent
        distance += current_node->distance(*current_node->parent);
        
        // traverse one node up the path
        current_node = current_node->parent;
    }

    // add the start node to the path
    path_found.insert(path_found.begin(), *current_node);

    // scale the total path distance
    distance *= m_Model.MetricScale();
   
    return path_found;
}

// TODO 7: Write the A* Search algorithm here.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

    // begin search by adding the start node to the open list
    open_list.push_back(start_node);
    start_node->visited = true;
    
    // using A*search, traverse the map from the start node to the end node
    current_node = start_node; 
    while (current_node != end_node) {
        // add the neighbors of the current node
        AddNeighbors(current_node);

        // select the node with the lowest f-value as the next node to traverse
        current_node = NextNode();
    }

    // build and store the final path of nodes that identify  
    // the shortest route between the start node and end nodes 
    m_Model.path = ConstructFinalPath(current_node);

    return ;
}
