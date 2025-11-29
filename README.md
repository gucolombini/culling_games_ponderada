# Culling Games Map Solver
Uma aplicação que resolve mapas gerados proceduralmente de forma completamente autônoma! É capaz de encontrar o menor caminho possível entre a posição inicial do robô e o alvo ao receber informação do mapa completo ou por meio de SLAM, com lógica de exploração e mapeamento por meio de Busca por Largura e Busca por Profundidade implementadas. O código, em C++, consiste na criação de um cliente capaz de se comunicar com as informações enviadas pela aplicação original por meio de tópicos e serviços ROS2, interpretando-as e se comunicando de volta com essa aplicação para fazer com que o robô se mova pelo mapa.

## [Vídeo de Demonstração Básica](https://drive.google.com/file/d/1BOMwQRBev4gHPaTdQctQSUhxAjYp3A40/view?usp=sharing)

# Instruções de Execução
No repositório [culling_games](https://github.com/rmnicola/culling_games) (compatível com última versão testada: 28/11/2025), dentro da pasta /src crie uma pasta chamada "map_solver". Copie e cole os conteúdos desse repositório dentro da pasta, e depois da raiz do repositório execute:
```bash
colcon build
```

Após o processo finalizar com sucesso, inicie o labirinto e em outro terminal execute os seguintes comandos para iniciar os algoritmos de resolvimento do mapa (Uma conexão estável é necessária para garantir a execução do programa sem erros!):

## Caminho mais curto por BFS (busca por largura)
```bash
source install/setup.bash
ros2 run map_solver complete_map_solver
```

## Mapeamento em tempo real por DFS (busca por profundidade) e BFS para backtracking
```bash
source install/setup.bash
ros2 run map_solver mapper_solver
```
#### Versão com estatísticas de precisão (faz uso do serviço /get_map para validar o mapeamento):
```bash
source install/setup.bash
ros2 run map_solver mapper_solver_stats
```

## Movimentação sem lógica (randomizada)
```bash
source install/setup.bash
ros2 run map_solver lobotomy_solver
```
