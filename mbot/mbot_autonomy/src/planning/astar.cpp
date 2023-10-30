#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

mbot_lcm_msgs::robot_path_t search_for_path(mbot_lcm_msgs::pose_xyt_t start,
                                             mbot_lcm_msgs::pose_xyt_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    // cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    //  ////////////////// TODO: Implement your A* search here //////////////////////////
    // mbot_lcm_msgs::robot_path_t path;
    // return path;

    Point<double> start_point;
    start_point.x = start.x;
    start_point.y = start.y;
    cell_t start_cell = global_position_to_grid_cell(start_point, distances);

    Point<double> end_point;
    end_point.x = goal.x;
    end_point.y = goal.y;
    cell_t end_cell = global_position_to_grid_cell(end_point, distances);

    mbot_lcm_msgs::robot_path_t path;
    path.utime = start.utime;
    //path.path.push_back(start);    
    //path.path_length = path.path.size();


    
    if (!distances.isCellInGrid(start_cell.x, start_cell.y)) {// && distances(start.x, start.y) > params.minDistanceToObstacle) {
        path.path.push_back(start);
        path.path_length = path.path.size();
        return path;
    }
    
    if (!distances.isCellInGrid(end_cell.x, end_cell.y)) {// && distances(goal.x, goal.y) > params.minDistanceToObstacle) {
        path.path.push_back(start);
        path.path_length = path.path.size();
        return path;
    }
    

    PriorityQueue open_list;
    std::vector<Node*> closed_list;
    

    Node* startNode = new Node(start_cell.x, start_cell.y);
    startNode->g_cost = 0;
    startNode->h_cost = 0;

    open_list.push(startNode);
    size_t iter = 0;

    while (!open_list.empty() && iter < params.maxIter) 
    {
        

        Node* node = open_list.pop();
        closed_list.push_back(node);

      


        //if reached goal
        if (node->cell == end_cell) {
            std::vector<Node*> nodePath = extract_node_path(node, startNode);
            //prune?
            
            path.path = extract_pose_path(nodePath, distances);
            path.path_length = path.path.size();

            //printf("path size = %d\n", path.path_length);
            //printf("NUM ITERS=%d\n", iter);

            break;
        }

        //add/update neighbors in closed list
        std::vector<Node*> neighbors = expand_node(node, distances, params);
        for (size_t i = 0; i < neighbors.size(); ++i) {

            //float g = g_cost(node, neighbors[i], distances, params);
            float g = g_cost(node, neighbors[i], distances, params);
            float h = h_cost(neighbors[i], end_cell, distances);

            Node *temp = get_from_list(neighbors[i], closed_list);
            if (temp) {
                if (temp->f_cost() < neighbors[i]->f_cost()) {
                    delete neighbors[i];
                    continue;
                }
            }

            temp = get_from_list(neighbors[i], open_list.elements);
            if (temp) {
                if (temp->f_cost() > neighbors[i]->f_cost()) {
                    temp->g_cost = g;
                    temp->h_cost = h;
                    temp->parent = node;
                }
            }
            else {
                neighbors[i]->g_cost = g;
                neighbors[i]->h_cost = h;
                neighbors[i]->parent = node;
                open_list.push(neighbors[i]);
            }
        }
        iter++;    
    }
    path.path_length = path.path.size();
    return path;

}



double h_cost(Node* from, cell_t goal, const ObstacleDistanceGrid& distances)
{
    // TODO: Return calculated h cost
    int dx = abs(from->cell.x - goal.x);
    int dy = abs(from->cell.y - goal.y);

    double h_cost = (dx + dy) + (1.414 - 2) * std::min(dx, dy);
    return h_cost;
}
double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    float dx = std::abs(from->cell.x - goal->cell.x);
    float dy = std::abs(from->cell.y - goal->cell.y);

    float g_cost = from->g_cost;

    if (dx + dy == 2) {
        g_cost += 1.414;
    } else {
        g_cost += 1;
    }
    ///< The exponent to apply to the distance cost, whose function is:
    ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
    ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
    
    float obstacle_cost = 0.0;
    float dist = distances(goal->cell.x, goal->cell.y); 
    if (dist > params.minDistanceToObstacle && dist < params.maxDistanceWithCost) {
        obstacle_cost = std::pow(params.maxDistanceWithCost - dist, params.distanceCostExponent);
        //std::cout << obstacle_cost << std::endl;
    }

    // is this right? accumulate obstacle cost?
    return g_cost + obstacle_cost;

}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return children of a given node that are not obstaclescd
    std::vector<Node*> children;
    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};

    for (int n = 0; n < 8; ++n) {
        
        cell_t adjacentCell(node->cell.x + xDeltas[n], node->cell.y + yDeltas[n]);
        
        if (distances.isCellInGrid(adjacentCell.x, adjacentCell.y) && distances(adjacentCell.x, adjacentCell.y) > params.minDistanceToObstacle) {
            Node *neighbor = new Node(adjacentCell.x, adjacentCell.y);
            neighbor->parent = node;
            children.push_back(neighbor);
        }
    }

    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    // TODO: Generate path by following parent nodes
    std::vector<Node*> path;
    Node* cur = goal_node;

    while (cur->parent != NULL) {
        path.push_back(cur);
        cur = cur->parent;
    }
    path.push_back(cur);

    if (cur->cell != start_node->cell) {
        std::cout << "PATH IS BROKEN" << std::endl;
        throw;
    }

    std::reverse(path.begin(), path.end());

    return path;

}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    // TODO: prune the path to generate sparse waypoints
    std::vector<mbot_lcm_msgs::pose_xyt_t> path;
    for (size_t i = 0; i < nodes.size(); ++i) {
        Point<double> pos = grid_position_to_global_position(nodes[i]->cell, distances);
        mbot_lcm_msgs::pose_xyt_t pose;
        pose.x = pos.x;
        pose.y = pos.y;
        pose.theta = 0.0;

        path.push_back(pose);
    }


    //clean path
    std::vector<mbot_lcm_msgs::pose_xyt_t> path1;
    std::vector<size_t> index_to_remove;
    int index_offset = 0;

    //remove intermediate points
    for (size_t i = 1; i < path.size() - 1; ++i) {
        mbot_lcm_msgs::pose_xyt_t curr = path[i];
        mbot_lcm_msgs::pose_xyt_t prev = path[i-1];
        mbot_lcm_msgs::pose_xyt_t next = path[i+1];

        float prev_ang = std::atan2(curr.y - prev.y, curr.x - prev.x);
        float next_ang = std::atan2(next.y - curr.y, next.x - curr.x);

        if (std::abs(prev_ang - next_ang) < 0.001) {
            index_to_remove.push_back(i);
        }
    }
    for (size_t i = 0; i < path.size(); ++i) {
        if (std::find(index_to_remove.begin(), index_to_remove.end(), i) == index_to_remove.end()) {
            path1.push_back(path[i]);
        }
    }


    //make path more sparse
    std::vector<mbot_lcm_msgs::pose_xyt_t> path2;
    index_to_remove.clear();
    for (size_t i = 1; i < path1.size(); ++i) {
        mbot_lcm_msgs::pose_xyt_t curr = path1[i];
        mbot_lcm_msgs::pose_xyt_t prev = path1[i-1];


        float dist = std::sqrt(std::pow(curr.x - prev.x, 2) + std::pow(curr.y - prev.y, 2));

        if (dist < 0.04) {
            index_to_remove.push_back(i);
        }
    }
    for (size_t i = 0; i < path1.size(); ++i) {
        if (std::find(index_to_remove.begin(), index_to_remove.end(), i) == index_to_remove.end()) {
            path2.push_back(path1[i]);
        }
    }

    //fill thetas
    for (size_t i = 1; i < path2.size()-1; ++i) {
        mbot_lcm_msgs::pose_xyt_t curr = path1[i];
        mbot_lcm_msgs::pose_xyt_t next = path1[i+1];


        float dx = next.x - curr.x;
        float dy = next.y - curr.y;
        float rad = std::atan2(dy, dx);

        path2[i].theta = rad;

    }


    return path2;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;
    
}
