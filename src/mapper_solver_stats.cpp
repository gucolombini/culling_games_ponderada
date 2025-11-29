#include <functional>
#include <memory>
#include <iostream>
#include <queue>
#include <stack>
#include <vector>
#include <algorithm>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"

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

/* Controlador do robô, faz uso do tópico de sensores e do serviço de movimento para se localizar.
Com essa implementação eu não preciso me preocupar do robô se perder no mapa :D */
class RobotController : public rclcpp::Node
{
public:
    std::stack<Position> movement_stack;
    std::vector<std::vector<bool>> discovered_map;
    bool is_searching = true;
    bool finished = false;
    bool received_first_msg = false;
    cg_interfaces::msg::RobotSensors last_msg;

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
        discovered_map.resize(int(known_map.size()), std::vector<bool>(int(known_map[0].size())));

        for (int r = 0; r < int(discovered_map.size()); r++) {
            for (int c = 0; c < int(discovered_map[0].size()); c++) {
                discovered_map[r][c] = false;
            }
        }

        discovered_map[robot.r][robot.c] = true;
        
        sub_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors",
            rclcpp::SensorDataQoS(),
            std::bind(&RobotController::callback, this, _1)
        );

        // Executa função do loop principal em outro thread para não afetar o recebimento de mensagens
        // Eu acho que deixei tudo bem seguro, quando o controller for apagado esse loop já irá ter parado
        std::thread t(&RobotController::main_loop, this);
        t.detach();

    }

    /* Atualiza o mapa de acordo com a mensagem recebida.
    Não faz checagem de bounds do mapa porque sempre há uma borda.
    Não bote mapa sem borda por favor Nicola vai explodir tudo*/
    void update_map() {
        known_map_[robot_.r-1][robot_.c-1] = last_msg.up_left[0];
        known_map_[robot_.r-1][robot_.c] = last_msg.up[0];
        known_map_[robot_.r-1][robot_.c+1] = last_msg.up_right[0];
        known_map_[robot_.r][robot_.c-1] = last_msg.left[0];
        known_map_[robot_.r][robot_.c] = 'R';
        known_map_[robot_.r][robot_.c+1] = last_msg.right[0];
        known_map_[robot_.r+1][robot_.c-1] = last_msg.down_left[0];
        known_map_[robot_.r+1][robot_.c] = last_msg.down[0];
        known_map_[robot_.r+1][robot_.c+1] = last_msg.down_right[0];
        std::cout << "UPDATED MAP\n";
        print_map(known_map_);
        known_map_[robot_.r][robot_.c] = 'f';
    }

    /* Atualiza o stack do DFS com os quadrados adjacentes ao robô */
    void update_DFS() {
        push_DFS(Position {r:robot_.r-1, c:robot_.c  });
        push_DFS(Position {r:robot_.r+1, c:robot_.c  });
        push_DFS(Position {r:robot_.r  , c:robot_.c-1});
        push_DFS(Position {r:robot_.r  , c:robot_.c+1});
    }

    /* Valida se o quadrante já foi descoberto
    Se não, adiciona ao stack 
    Se sim, ignora.*/
    void push_DFS(Position pos) {
        if (discovered_map[pos.r][pos.c]) return;
        if (known_map_[pos.r][pos.c] == 'f') {
            discovered_map[pos.r][pos.c] = true;
            movement_stack.push(pos);
        }
    }

    /*Função de movimento principal do robô.
    Retorna true se o movimento teve sucesso, e false caso contrário.
    Atualiza a posição do robô automaticamente*/
    bool move_logic(std::string dir){
        auto [new_r, new_c] = send_move(main_node_, move_client_, dir); 
        if (new_r == -1) { 
            RCLCPP_WARN(main_node_->get_logger(), "Move failed"); return false; 
        } 
        robot_.r = new_r; robot_.c = new_c; 
        return true;
    }

    /*Move o robô dentro do caminho especificado o mais rápido possível.
    Não espera o mapa atualizar porque não dá tempo de fazer isso.
    Recomendável desativar a atualização do mapa enquanto roda essa função.*/
    bool follow_path(std::vector<Position> path) {
        if (path.empty()) {
            std::cout << "Caminho vazio";
            return false;
        }
        is_searching = false;
        for (int n = 0; n < int(path.size());) {
            bool success = false;
            if      (path[n].r == robot_.r - 1) {success = move_logic("up");}
            else if (path[n].r == robot_.r + 1) {success = move_logic("down");}
            else if (path[n].c == robot_.c - 1) {success = move_logic("left");}
            else if (path[n].c == robot_.c + 1) {success = move_logic("right");}
            else if (path[n] == robot_) {success = true;}
            else {
                std::cout << "Caminho inválido";
                is_searching = true;
                return false;}
            if (success) {
                ++n;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        is_searching = true;
        return true;
    }

private:
    void callback(const cg_interfaces::msg::RobotSensors &msg)
    {
        last_msg = msg;
        received_first_msg = true;
    }

    /* Função usada para finalizar a função do main loop sem quebrar tudo.
    Por estar em um thread separado, é importante finalizá-a de forma apropriada
    para que não tente acessar variáveis do controller após ele ser apagado*/
    void finish_main() {
        is_searching = false;
        finished = true;
        std::cout << "E no fim das contas, o verdadeiro fim do labirinto eram os amigos que fizemos pelo caminho.";
        if (!follow_path(BFS(known_map_, robot_, Position{1, 1}))) {
            // Se falhar o pathfinding ele morre.
            rclcpp::shutdown(); 
            return;
        };

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

        follow_path(shortest_path);

        rclcpp::shutdown(); 
        return;
    }

    void main_loop() {
        // Lógica principal!!! Usando DFS iterativo para explorar o mapa
        // 1. Atualizar mapa com nova mensagem
        // 2. Atualizar o stack DFS com as novas informações
        // 3. Se o stack estiver vazio, interromper exploração
        // 4. Mover o robô até a posição no topo do stack
        while (!finished) {
            if (!is_searching || !received_first_msg) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            update_map();
            update_DFS();

            if (movement_stack.empty()) {
                finish_main();
                return;
            };

            Position next = movement_stack.top();

            bool success = false;
            if      (next.r == robot_.r - 1 && next.c == robot_.c    ) success = move_logic("up"   );
            else if (next.r == robot_.r + 1 && next.c == robot_.c    ) success = move_logic("down" );
            else if (next.r == robot_.r     && next.c == robot_.c - 1) success = move_logic("left" );
            else if (next.r == robot_.r     && next.c == robot_.c + 1) success = move_logic("right");
            else if (next == robot_) success = true;
            else {
                if (!follow_path(BFS(known_map_, robot_, next))) {
                    finish_main(); 
                    return;
                }
            }
            if (success) movement_stack.pop();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    rclcpp::Node::SharedPtr main_node_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    Position &robot_;
    std::vector<std::vector<char>> &known_map_;

    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mapper_solver_client");

    // Lógica para conseguir o mapa real, usa de referência na criação do mapa vazio e na validação do mapa escaneado
    auto client = node->create_client<cg_interfaces::srv::GetMap>("/get_map");
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Serviço /get_map indisponível.");
        return 1;
    }
    auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
    auto future = client->async_send_request(request);
    
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
    std::vector<std::vector<char>> true_map(rows, std::vector<char>(cols));
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            true_map[r][c] = flat[r * cols + c][0]; // converte string pra chars individuais
        }
    }
    std::cout << "MAPA REAL:\n";
    print_map(true_map);

    std::vector<std::vector<char>> known_map(rows, std::vector<char>(cols));

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            known_map[r][c] = 'u';
        }
    }

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
        known_map
    );
    rclcpp::spin(controller);

    if (!true_map.empty()) {
        int total_spaces = 0;
        int discovered_spaces = 0;
        int accurate_spaces = 0;
        for (int r = 0; r < int(known_map.size()); r++) {
            for (int c = 0; c < int(known_map[0].size()); c++) {
                ++total_spaces;
                if (known_map[r][c] != 'u') ++discovered_spaces;
                if (known_map[r][c] == true_map[r][c] || (known_map[r][c] == 'f' && true_map[r][c] == 'r')) ++accurate_spaces;
            }
        }
        std::cout << "\n\n MAPA DESCOBERTO             : ";
        std::cout << (double(discovered_spaces) / double(total_spaces)) * 100;
        std::cout << "%";
        std::cout << "\n PRECISÃO (APENAS DESCOBERTO): ";
        std::cout << (double(accurate_spaces) / double(discovered_spaces)) * 100;
        std::cout << "%";

        Position true_target = find_target(true_map);
        Position known_target = find_target(known_map);
        Position start_position = {r:1, c:1};

        if (true_target == known_target) {
            // Se o alvo for o mesmo, podemos comparar os caminhos!
            std::vector<Position> true_path = BFS(true_map, start_position, true_target);
            std::vector<Position> known_path = BFS(known_map, start_position, known_target);

            std::cout << "\n\n CAMINHO MAIS CURTO ENCONTRADO? ";
            if (true_path == known_path) {
                std::cout << "\n SIM!!!!!!!!!!";
            } else if (true_path.size() == known_path.size()) {
                std::cout << "\n TALVEZ!!!!!!!! Caminhos diferentes mas de mesmo tamanho! (provavelmente ocorreu algum erro no mapeamento)";
            } else {
                std::cout << "\n Não.";
            }
        } else {
            std::cout << "\n\n ALVO ENCONTRADO É INCORRETO, IMPOSSÍVEL COMPARAR :(";
        }
        std::cout << "\n";
    } else {
        std::cout << "Mapa real indisponível, não é possível compará-los";
    }

    rclcpp::shutdown();
    return 0;
}