# Instruções de Execução
No repositório [culling_games](https://github.com/rmnicola/culling_games), dentro da pasta /src crie uma pasta chamada "map_solver". Copie e cole os conteúdos desse repositório dentro da pasta, e depois da raiz do repositório execute:
```bash
colcon build --packages-select map_solver
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
