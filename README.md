# SuperMegaBot_SMB
ETH 4 wheel Super Mega Bot (SMB) applications [https://ethz-robotx.github.io/SuperMegaBot/]


# Projeto de Implementação 1 - smb_common

Neste repositório se encontra o pacote smb_common utilizado para a execução do Projeto de Implementação 1, da disciplina Robôs Móveis Autônomos (RMA) - DC UFSCar

## Reconhecimento e créditos

As ideias de implementação do algorítmo Wavefront foram obtidas a partir do material do canal OneLoneCoder. Seu repositório está disponível [aqui](https://github.com/OneLoneCoder/olcPixelGameEngine/blob/master/Videos/OneLoneCoder_PGE_PathFinding_WaveProp.cpp).

## Dependências

Além dos pacotes obtidos durante a execução da disciplina, as dependências dos códigos python são:

* numpy v.1.22.1
* opencv-python v.4.2.0.34
* pandas v.1.0.4

As dependências python são verificadas na etapa de instalação a seguir

## Instalação

Para a instalação desse pacote, é necessário criar um novo workspace para a instalação e construção do ambiente de simulação.
Os passos a seguir realizam a instalação do smb_common do projeto:

Criação do workspace:
``` sh
mkdir -p ~/workspace_projeto/src
cd ~/workspace_projeto/
catkin_make
source devel/setup.bash
catkin init
catkin clean
```

Instalação de dependências
``` sh
cd ~/workspace_projeto/src
git clone https://github.com/ros-perception/slam_gmapping.git
git clone https://github.com/ros-planning/navigation.git
git clone https://github.com/joaocarloscampi/smb_common
catkin build
cd smb_common
pip3 install -r requirements.txt
```

## Execução

Para a execução do código, é necessário verificar algumas configurações antes de qualquer coisa. 

Primeiro, precisamos verificar os seguintes itens no arquivo smb_gazebo/launch/projeto_rma.launch

* Verifique se o ponto de spawn do robô é o mesmo do início do seu path. Esse spawn é feito nos argumentos x, y e z do início do arquivo (linhas 22, 23 e 24);
* O endereço do arquivo .csv que armazena o path gerado é passado no parâmetro path_map, na execução do nó pathFinder. Assegure que o endereço é o mesmo do arquivo gerado pelo código python em smb_gazebo/src/processamento_csv.py .

Após isso, precisamos gerar o path para o robô. Isso é feito pelo arquivo smb_gazebo/src/processamento_csv.py. Verifique se:
* O arquivo de mapa ***mapa.pgm*** está nesse diretório;
* As coordenadas de início do path no início do código é o mesmo do spawn do robô em smb_gazebo/launch/projeto_rma.launch;
* A coordenada de fim do path está correta no início do código.

Após isso, execute o seguinte código para gerar os mapas segmentados e o arquivo .csv com o path:
``` sh
roscd smb_gazebo/src
python3 processamento_csv.py
```
Por fim, basta executar o launch desse pacote:
``` sh
roscd smb_gazebo/launch
roslaunch projeto_rma.launch
```

## Vídeo da execução

Para acessar o vídeo da execução do projeto com o robô atingindo o ponto desejado, clique [aqui](https://drive.google.com/file/d/1W6UDqgpirMCnvRqb7XizbO5vyU9BOG-L/view?usp=sharing).




## 📷 Screenshots

<p align="center">
  <img height=200px src="./docs/mapping/Screenshot (447).png" />
  <img height=200px src="./docs/mapping/Screenshot (449).png" />
  <img height=200px src="./docs/mapping/Screenshot (450).png" />
  <img height=200px src="./docs/mapping/mob_robM_2023_04_03__02_14_58.png" />
</p>

## 🎈 Intro

It is necessary to clone the project inside a catkin workspace. To create a workspace, refer to [this link](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

To run the simulation with one controllable robot

```bash
roslaunch travesim simulation_robot.launch
```

To run the simulation with the entire team

```bash
roslaunch travesim simulation_team.launch
```

To run the simulation of a match

```bash
roslaunch travesim simulation_match.launch
```

## 🌎 Worlds

TraveSim can handle simulating games with 3 or 5 robots per team. The number of robots per team will be inferred from the chosen simulation world. The worlds currently supported are as follows:

- `vss_field.world` - Base world for 3x3 matches
- `vss_field_camera.world` - World for 3x3 matches with camera and spotlights
- `vss_field_5.world` - Base world for 5x5 matches

So, for example, to run the simulation with a single team of 5 robots, run:

```bash
roslaunch travesim simulation_team.launch world_name:=vss_field_5.world
```

For more information about roslaunch parameters, see the [🚀 Roslaunch](#-roslaunch) section.

## 📣 ROS topics

### ⬅ Input

The simulation can work using 2 input interfaces, **differential drive control** (default) or **direct motor control**. It is important to notice that is not possible to use both interfaces to control different robots at the same time.

#### Differential drive control (default)

By default, the simulation receives commands of type [geometry_msgs/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html), representing the desired velocity of the robot in two components: linear and angular.

```python
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
```

The ROS topics follow the naming convention:

- **/yellow_team/robot_[0..2|0..4]/diff_drive_controller/cmd_vel**
- **/blue_team/robot_[0..2|0..4]/diff_drive_controller/cmd_vel**

The control of the robot is performed by the [diff_driver_controller](http://wiki.ros.org/diff_drive_controller) from the library [ros_control](http://wiki.ros.org/ros_control). The controller represents the behavior of the embedded system of the robot and will send torque commands to the motors in order to follow the received set point.

The parameters of this controller are specified in the file [./config/motor_diff_drive.yml](./config/motor_diff_drive.yml).

#### Direct motor control

The simulation also accepts control directly over **angular velocity** commands (in rad/s) for both robot's motors (through the interface [velocity_controller](http://wiki.ros.org/velocity_controllers) from [ros_control](http://wiki.ros.org/ros_control)). This interface mimics a controller interface more coupled to the robots characteristics than differential drive control.

The commands are read from topics of type [std_msgs/Float64](http://docs.ros.org/noetic/api/std_msgs/html/msg/Float64.html), representing each motor's speed in **rad/s**

- **/yellow_team/robot_[0..2|0..4]/left_controller/command**
- **/yellow_team/robot_[0..2|0..4]/right_controller/command**
- **/blue_team/robot_[0..2|0..4]/left_controller/command**
- **/blue_team/robot_[0..2|0..4]/right_controller/command**

In order to enable this control interface, one should send the parameter `twist_interface` as false in roslaunch [parameters](#-parameters)

The parameters of this controller are specified in the file [./config/motor_direct_drive.yml](./config/motor_direct_drive.yml).

### ➡ Output

By default, Gazebo publishes in the topic **/gazebo/model_states** of type [gazebo_msgs/ModelStates](http://docs.ros.org/melodic/api/gazebo_msgs/html/msg/ModelStates.html), with an array of informations about each model in the simulation.

```python
# broadcast all model states in world frame
string[] name                 # model names
geometry_msgs/Pose[] pose     # desired pose in world frame
geometry_msgs/Twist[] twist   # desired twist in world frame
```

For convenience, this package have a script ([vision_proxy.py](./scripts/vision_proxy.py)) that subscribes this topic and republishes the information at different topics of type [gazebo_msgs/ModelState](http://docs.ros.org/melodic/api/gazebo_msgs/html/msg/ModelState.html) for each entity (3 yellow team robots, 3 blue team robots and 1 ball, 7 in total).

```python
# Set Gazebo Model pose and twist
string model_name           # model to set state (pose and twist)
geometry_msgs/Pose pose     # desired pose in reference frame
geometry_msgs/Twist twist   # desired twist in reference frame
string reference_frame      # set pose/twist relative to the frame of this entity (Body/Model)
                            # leave empty or "world" or "map" defaults to world-frame
```

The republished topics are

- **/vision/yellow_team/robot_[0..2|0..4]** - Yellow team robots's topics
- **/vision/blue_team/robot_[0..2|0..4]** - Blue team robots's topics
- **/vision/ball** - Ball's topic


## References

https://ethz-robotx.github.io/SuperMegaBot/core-software/installation_core.html

https://ethz-robotx.github.io/SuperMegaBot/core-software/HowToRunSoftware.html

https://unlimited.ethz.ch/display/ROBOTX/SuperMegaBot

https://github.com/ethz-asl/eth_supermegabot

https://github.com/cra-ros-pkg/robot_localization/blob/noetic-devel/params/ekf_template.yaml