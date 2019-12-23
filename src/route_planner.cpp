#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}




float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}




void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(RouteModel::Node* n : current_node->neighbors)
    {
        n->parent = current_node;
        n->h_value = CalculateHValue(n);
        n->g_value = n->distance(*current_node) + current_node->g_value;
        open_list.push_back(n);
        n->visited = true;
    }
}



bool Compare(RouteModel::Node* a, RouteModel::Node* b)
{
    const float f1 = a->h_value + a->g_value;
    const float f2 = b->h_value + b->g_value;
    return f1 > f2;
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(std::begin(open_list), std::end(open_list), Compare);
    RouteModel::Node* next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}




std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    RouteModel::Node parent;

    while(current_node->parent != nullptr)
    {
        path_found.push_back(*current_node);
        parent = *(current_node->parent);
        distance += current_node->distance(parent);
        current_node = current_node->parent;
    }

    path_found.push_back(parent);
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}



void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = start_node;
    current_node->g_value = 0.0f;
    open_list.push_back(current_node);
    current_node->visited = true;

    while(current_node->x != end_node->x && current_node->y != end_node->y)
    {
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    m_Model.path = ConstructFinalPath(current_node);

}
