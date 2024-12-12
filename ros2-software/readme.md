# Passos para Execução do Projeto no Ambiente Gazebo

## 1. Ativação do ambiente

Antes de executar qualquer comando, é necessário ativar o ambiente configurado:

```bash
source install/setup.bash
```

## 2. Execução do Gazebo

Inicie o ambiente de simulação Gazebo com o modelo do robô carregado atráves do arquivo launch configurado:

```bash
ros2 launch robot_description gazebo.launch.py 
```

## 3. Execução dos pacotes

Para que o robô funcione corretamente, é necessário iniciar os pacotes responsáveis pelo controle, localização e tomada de decisão. Execute os comandos abaixo em terminais separados:

```bash
ros2 launch robot_controller controller.launch.py 
```

```bash
ros2 launch my_robot_localization local_localization.launch.py 
```

```bash
ros2 launch robot_decision_making decision_making.launch.py
```

## 4. Ferramentas de visualização

Para visualizar o robô e os dados de sensores no ambiente gráfico do RViz 2:

```bash
ros2 launch robot_description display.launch.py 
```

Para monitorar os dados dos tópicos em tempo real de forma gráfica, utilize o `PlotJuggler`:

```bash
ros2 run plotjuggler plotjuggler 
```

## 5. Enviar comandos para o robô

Para ativar a detecção de obstáculos no robô, ajuste o parâmetro relacionado ao uso do sensor ultrassônico:

```bash
ros2 param set /robot_controller no_ultrassonic 0
```

Envie um comando para que o robô se mova até uma posição desejada:

```bash
ros2 topic pub /robot/desired_pose geometry_msgs/msg/Pose "position:
  x: 5.0
  y: 0.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0" --once
```

---

Siga os passos acima na ordem indicada para configurar e operar o projeto corretamente no ambiente de simulação Gazebo.

