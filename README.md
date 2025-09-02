# Dynamic Follower – Simulação de gestos dinâmicos e interação com ROS2

Este projeto implementa um sistema de navegação para robôs baseado em ROS 2, permitindo que um robô siga pontos dinâmicos (ex.: uma pessoa em movimento) em um ambiente simulado.  
A arquitetura integra **simulação com Pygame**, **ROS 2 Navigation2**, **Behavior Trees personalizadas** e **publicação dinâmica de objetivos**.

---

## 📌 Estrutura do Repositório

```

dynamic\_follower/
├── README.md                   
├── ros2_ws/                   
│   └── src/
│       └── dynamic_follower/
│           └── dynamic_follower/
│               ├── follower_node.py   
│               ├── __init__.py
├── navigation2/               
│   ├── Dockerfile
│   ├── nav2-rviz2-container.sh
│   ├── README.md
│   ├── behavior_trees/         
│   │   └── follow_dynamic_point.xml
│   ├── maps/                   
│   │   ├── labvisio_corredor_sync.pgm
│   │   ├── labvisio_corredor_sync.yaml
│   │   ├── map_cam_odom_exp_170601.pgm
│   │   └── map_cam_odom_exp_170601.yaml
│   ├── nav2/params/nav2_params.yaml    
│   ├── robot_localization/params/      
│   │   ├── ekf.yaml
│   │   ├── ekf_fuse.yaml
│   │   └── ukf.yaml
│   └── rviz_config/             
│       ├── fuse.rviz
│       ├── map.rviz
│       └── map_odometry.rviz
└── position_sender/             
├── app.py                   
├── is_to_ros.py             
├── ros_pb2.py               

````

---

## 📖 Visão Geral

O sistema é composto por três módulos principais:

1. **Simulação (position_sender/)**
   - Interface feita com **Pygame** para simular uma pessoa andando.
   - Captura de comandos de movimento (WASD) e gestos (P, V, F).
   - Envio de dados para ROS via gateway.

2. **ROS 2 Workspace (ros2_ws/)**
   - Contém o nó **`follower_node.py`**, que publica objetivos de navegação no tópico `/goal_pose`.
   - Pode ser expandido para integração com Nav2 Behavior Trees.

3. **Navigation2 Config (navigation2/)**
   - Configuração do Navigation2 (mapas, parâmetros, Behavior Trees).
   - Behavior Tree personalizada **`follow_dynamic_point.xml`** para seguir pontos dinâmicos.

---

## 🔗 Dependências

### Python:
- **pygame**
- **protobuf**
- **is_wire**

### ROS 2:
- Navigation2 (`nav2_bringup`)
- Behavior Trees (`bt_navigator`)
- Pacote `robot_localization`

### Gateway:
- [is-ros2-gateway](https://github.com/labvisio/is-ros2-gateway)

---

## ⚙️ Instalação

### 1. Clone este repositório

```bash
git clone https://github.com/AntonioSTO/dynamic_follower.git
cd dynamic_follower
````

### 2. Instale dependências Python

```bash
cd position_sender
pip install -r requirements.txt
```

### 3. Compile o workspace ROS 2

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

---

## ▶️ Como Executar

### **Passo 1 – Iniciar Navigation2**

```bash
source /opt/ros/humble/setup.bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true
```

### **Passo 2 – Iniciar o nó seguidor**

```bash
source install/setup.bash
ros2 run dynamic_follower follower_node
```

### **Passo 3 – Rodar a simulação Pygame**

```bash
cd position_sender
python3 app.py
```

**Controles da simulação:**

* **WASD** → mover no grid
* **P** → parar
* **V** → venha
* **F** → siga

---

## ✅ Exemplo de Fluxo Completo

```bash
# Terminal 1
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true

# Terminal 2
ros2 run dynamic_follower follower_node

# Terminal 3
python3 position_sender/app.py
```

---

## 📚 Referências

* [ROS 2 Documentation](https://docs.ros.org/en/)
* [Navigation2](https://navigation.ros.org/)
* [is-ros2-gateway](https://github.com/labvisio/is-ros2-gateway)
