#include <functional>
#include <memory>
#include <iostream>
#include <queue>
#include <stack>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

using std::placeholders::_1;

/* Estrutura de posição 
r: linha, c: coluna*/
struct Position {
    int r, c;
    bool operator==(const Position &other) const {
        return r == other.r && c == other.c;
    }
};

/* Função de Busca em Largura (Breadth First Search).
Retorna lista das posições que formam a rota do ponto inicial até o alvo.
Usada para descobrir o menor caminho possível em um mapa já conhecido */
std::vector<Position> BFS(std::vector<std::vector<char>> map, Position start, Position target) {
    int rows = int(map.size());
    int cols = int(map[0].size());

    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<Position>> parent(rows, std::vector<Position>(cols, {-1, -1}));

    std::queue<Position> q;
    q.push(start);
    visited[start.r][start.c] = true;

    while (!q.empty()) {
        Position cur = q.front();
        q.pop();

        if (Position {cur.r, cur.c} == target) {

            // Reconstrói o caminho até o alvo
            std::vector<Position> path;
            Position p = cur;

            while (!(p.r == start.r && p.c == start.c)) {
                path.push_back(p);
                p = parent[p.r][p.c];
            }
            path.push_back({start.r, start.c});
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Explora quadrados adjacentes
        for (int i = 0; i < 4; i++) {
            int nr = cur.r;
            int nc = cur.c;

            if (i == 0) --nr;       // up
            else if (i == 1) ++nr;  // down
            else if (i == 2) --nc;  // left
            else if (i == 3) ++nc;  // right

            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
            if (map[nr][nc] == 'b' || map[nr][nc] == 'u') continue; // Ignorar paredes, espaços desconhecidos
            if (map[nr][nc] == 't' && !(Position {nr, nc} == (target))) continue; // Ignorar alvo EXCETO quando ele é a posição alvo
            if (visited[nr][nc]) continue;

            visited[nr][nc] = true;
            parent[nr][nc] = cur;
            q.push({nr, nc});
        }
    }
    std::vector<Position> empty;
    return empty;
}

/* Exibe o mapa especifcado no terminal */
void print_map(const std::vector<std::vector<char>> &map)
{
    for (const auto &row : map)
    {
        for (char cell : row)
            std::cout << cell;
        std::cout << '\n';
    }
    std::cout << '\n';
}

/* Retorna posição do alvo em um mapa.
Se nenhum for encontrado, retorna {-1, -1} */
Position find_target(const std::vector<std::vector<char>> &map) {
    Position target;
    target.r = -1; target.c = -1;
    for (int r = 0; r < int(map.size()); r++) {
        for (int c = 0; c < int(map[0].size()); c++) {
            if (map[r][c] == 't') {
                target.r = r; target.c = c; // Registra a coordenada do alvo para depois dizer a rota final.
                break;
            }
        }
    }
    return target;
}

std::pair<int,int> send_move(
    rclcpp::Node::SharedPtr node,
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
        return {-1, -1};   // caso de falha = posição impossível
    }

    auto result = future.get();
    return { result->robot_pos[0], result->robot_pos[1] };
}

/* Controlador do robô 
Possui uma função simples de movimento e uma de resolver o mapa*/
class RobotController : public rclcpp::Node
{
public:
    Position target = {-1,-1};
    RobotController(
        rclcpp::Node::SharedPtr main_node,
        rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client,
        Position &robot,
        std::vector<std::vector<char>> &known_map)
    : Node("robot_controller"),
      main_node_(main_node),
      move_client_(move_client),
      robot_(robot),
      known_map_(known_map)
    {
        solve();
    }

    /*Função de movimento principal do robô.
    Retorna true se o movimento teve sucesso, e false caso contrário.
    Atualiza a posição do robô automaticamente*/
    bool move_logic(std::string dir)
        {auto [new_r, new_c] = send_move(main_node_, move_client_, dir); 
        if (new_r == -1) { 
            RCLCPP_WARN(main_node_->get_logger(), "Move failed"); return false; 
        } 
        robot_.r = new_r; robot_.c = new_c; 
        return true;
    }

    /*Move o robô dentro do caminho especificado o mais rápido possível.*/
    bool follow_path(std::vector<Position> path) {
        if (path.empty()) {
            std::cout << "Caminho vazio";
            return false;
        }
        for (int n = 0; n < int(path.size());) {
            bool success = false;
            if (path[n].r == robot_.r - 1) {success = move_logic("up");}
            else if (path[n].r == robot_.r + 1) {success = move_logic("down");}
            else if (path[n].c == robot_.c - 1) {success = move_logic("left");}
            else if (path[n].c == robot_.c + 1) {success = move_logic("right");}
            else if (path[n] == robot_) {success = true;}
            else {
                std::cout << "Caminho inválido";
                return false;}
            if (success) {
                ++n;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return true;
    }

    /* Resolve o mapa especificado */
    void solve()
    {
        target = find_target(known_map_);

        auto shortest_path = BFS(known_map_, robot_, target);
        for (int i = 0; i < int(shortest_path.size()); i++) {
            std::cout << "\n STEP ";
            std::cout << i;
            std::cout << ": ";
            std::cout << shortest_path[i].r;
            std::cout << ", ";
            std::cout << shortest_path[i].c;
        }
        std::cout << "\n";

        follow_path(shortest_path);
        return;
    }

private:
    rclcpp::Node::SharedPtr main_node_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    Position &robot_;
    std::vector<std::vector<char>> &known_map_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("map_solver_client");

    // Create client
    auto client = node->create_client<cg_interfaces::srv::GetMap>("/get_map");

    // Wait for service
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Serviço /get_map indisponível.");
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
        RCLCPP_ERROR(node->get_logger(), "Falha em chamar o serviço /get_map");
        return 1;
    }

    auto response = future.get();

    const auto &flat = response->occupancy_grid_flattened;
    const auto &shape = response->occupancy_grid_shape;

    if (int(shape.size()) != 2) {
        RCLCPP_ERROR(node->get_logger(), "Mapa inválido!");
        return 1;
    }

    int rows = shape[0];
    int cols = shape[1];

    std::cout << "Mapa real recebido." << rows << "x" << cols << "\n";

    // guarda o mapa na memória
    std::vector<std::vector<char>> map(rows, std::vector<char>(cols));
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            map[r][c] = flat[r * cols + c][0]; // converte string pra chars individuais
        }
    }

    print_map(map);

    // Create move command client
    auto move_client = node->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

    // Wait for service
    if (!move_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Service /move_command not available.");
        return 1;
    }

    Position robot;
    robot.r = 1;
    robot.c = 1;

    // Instancia o controlador do robô
    auto controller = std::make_shared<RobotController>(
        node,
        move_client,
        robot,
        map
    );

    return 0;
}