#include <functional>
#include <memory>
#include <iostream>
#include <queue>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

struct Node {
    int r, c;
};

bool send_move(rclcpp::Node::SharedPtr node,
               rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client,
               const std::string &dir)
{
    auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    req->direction = dir;

    auto future = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node, future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call /move_command");
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "Move '%s' executed.", dir.c_str());
    return true;
}

std::vector<Node> BFS(std::vector<std::vector<char>> map, int start_r, int start_c) {
    int rows = map.size();
    int cols = map[0].size();

    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<Node>> parent(rows, std::vector<Node>(cols, {-1, -1}));

    std::queue<Node> q;
    q.push({start_r, start_c});
    visited[start_r][start_c] = true;

    while (!q.empty()) {
        Node cur = q.front();
        q.pop();

        // Found target
        if (map[cur.r][cur.c] == 't') {

            // ---- RECONSTRUCT PATH ----
            std::vector<Node> path;
            Node p = cur;

            while (!(p.r == start_r && p.c == start_c)) {
                path.push_back(p);
                p = parent[p.r][p.c];
            }
            path.push_back({start_r, start_c});

            std::reverse(path.begin(), path.end());

            std::cout << "PATH:\n";
            for (auto &n : path) {
                std::cout << "(" << n.r << ", " << n.c << ")\n";
            }

            return path;
        }

        // Explore neighbors
        for (int i = 0; i < 4; i++) {
            int nr = cur.r;
            int nc = cur.c;

            if (i == 0) --nr;       // up
            else if (i == 1) ++nr;  // down
            else if (i == 2) --nc;  // left
            else if (i == 3) ++nc;  // right

            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
            if (map[nr][nc] == 'b') continue;
            if (visited[nr][nc]) continue;

            visited[nr][nc] = true;
            parent[nr][nc] = cur;
            q.push({nr, nc});
        }
    }
    std::vector<Node> empty;
    return empty;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create node
    auto node = rclcpp::Node::make_shared("get_map_client");

    // Create client
    auto client = node->create_client<cg_interfaces::srv::GetMap>("/get_map");

    // Wait for service
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Service /get_map not available.");
        return 1;
    }

    // Create request
    auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();

    // Call service
    auto future = client->async_send_request(request);

    // Wait for response
    if (rclcpp::spin_until_future_complete(node, future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service /get_map");
        return 1;
    }

    auto response = future.get();

    const auto &flat = response->occupancy_grid_flattened;
    const auto &shape = response->occupancy_grid_shape;

    if (shape.size() != 2) {
        RCLCPP_ERROR(node->get_logger(), "Invalid shape");
        return 1;
    }

    int rows = shape[0];
    int cols = shape[1];

    std::cout << "Received map " << rows << "x" << cols << "\n";

    // Print as matrix
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            std::cout << flat[r * cols + c];
        }
        std::cout << "\n";
    }

    // guarda o mapa na memória
    std::vector<std::vector<char>> map2d(rows, std::vector<char>(cols));

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            map2d[r][c] = flat[r * cols + c][0]; // take the char inside the string
        }
    }

    // Create move command client
    auto move_client = node->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

    // Wait for service
    if (!move_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Service /move_command not available.");
        return 1;
    }
    
    std::cout << "\n";
    std::vector<Node> path = BFS(map2d, 1, 1);
    if (!path.empty()) {
        Node robot;
        robot.r = 1;
        robot.c = 1;
        for (auto &n : path) {
            if (n.r == robot.r - 1) {send_move(node, move_client, "up");}
            else if (n.r == robot.r + 1) {send_move(node, move_client, "down");}
            else if (n.c == robot.c - 1) {send_move(node, move_client, "left");}
            else if (n.c == robot.c + 1) {send_move(node, move_client, "right");}
            robot.r = n.r;
            robot.c = n.c;
            }
    }


    rclcpp::shutdown();
    return 0;
}