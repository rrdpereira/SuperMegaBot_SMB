# SuperMegaBot_SMB
Project based on ETHZ 4 wheel Super Mega Bot (SMB) applications [https://ethz-robotx.github.io/SuperMegaBot/]

## Intro 

Autonomous Mobile Robot (AMR) Project for CCO - 728 - Autonomous Mobile Robot (Robôs Móveis Autônomos) course of PPGCC UFSCAr [https://www.ppgcc.ufscar.br/pt-br/programa/estrutura-curricular/disciplinas-do-programa/cco-728-robos-moveis-autonomos]
# Projeto de Implementação 1 - smb_common

## Dependences

This package was implemented on Linux Ubuntu 20.04 Focal Fossa[link](https://releases.ubuntu.com/focal/) distribution using ROS1 Noetic Ninjemys[link](http://wiki.ros.org/noetic). The installation of the necessary ROS Noectic packages can be done with following bash command:

```bash
sudo apt-get install ros-noetic-amcl ros-noetic-costmap-converter ros-noetic-depthimage-to-laserscan ros-noetic-dynamic-reconfigure ros-noetic-ddynamic-reconfigure ros-noetic-ddynamic-reconfigure-dbgsym ros-noetic-ddynamic-reconfigure-python ros-noetic-geometry2 ros-noetic-hector-slam ros-noetic-move-base ros-noetic-move-base-flex ros-noetic-navigation ros-noetic-openslam-gmapping ros-noetic-rplidar-ros ros-noetic-slam-gmapping ros-noetic-spatio-temporal-voxel-layer ros-noetic-teb-local-planner ros-noetic-teleop-twist-keyboard ros-noetic-teleop-twist-joy ros-noetic-urg-node ros-noetic-rtabmap ros-noetic-rtabmap-ros ros-noetic-octomap ros-noetic-octomap-ros ros-noetic-octomap-rviz-plugins ros-noetic-octomap-server ros-noetic-octovis ros-noetic-imu-filter-madgwick ros-noetic-robot-localization ros-noetic-robot-pose-ekf ros-noetic-pointcloud-to-laserscan ros-noetic-rosbridge-server ros-noetic-map-server ros-noetic-realsense2-camera ros-noetic-realsense2-description ros-noetic-cmake-modules ros-noetic-velodyne-gazebo-plugins ros-noetic-ompl ros-noetic-navfn ros-noetic-dwa-local-planner ros-noetic-global-planner ros-noetic-costmap-2d ros-noetic-robot-self-filter ros-noetic-ros-numpy ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-grid-map-costmap-2d ros-noetic-grid-map-ros ros-noetic-grid-map-filters ros-noetic-grid-map-visualization ros-noetic-tf2-tools pcl-tools python3-vcstool python3-catkin-tools python3-catkin-lint python3-pip python3-rosdep ros-noetic-gazebo-plugins ros-noetic-hector-gazebo ros-noetic-hector-gazebo-plugins
```
After that, it is necessary to clone the project inside a catkin workspace. If there isn't a catkin workspace, you can create by this tutorial[link](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), or you can create using the following bash steps:

``` sh
# source ROS1 Noetic
source /opt/ros/noetic/setup.bash
# create the directories
mkdir -p ~/catkin_workspace/src
cd ~/catkin_workspace/
# initilize the catkin workspace
catkin init
catkin config --extend /opt/ros/noetic
catkin config -DCMAKE_BUILD_TYPE=Release
# navigate to the directory of src to clone SuperMegaBot_SMB project
cd ~/catkin_workspace/src
git clone https://github.com/rrdpereira/SuperMegaBot_SMB.git
# build the project
cd ~/catkin_workspace/
catkin build
# source your catkin worksapce
source ~/catkin_workspace/devel/setup.bash

# (optinal) may you found some errors, so you can use the "Magic" of rosdep
cd ~/catkin_workspace/src
rosdep install --from-paths . --ignore-src --os=ubuntu:focal -r -y
cd ~/catkin_workspace/
catkin build
# source your catkin worksapce
source ~/catkin_workspace/devel/setup.bash
```

## Run Teleoperation Mode

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

### Vídeo da execução

Para acessar o vídeo da execução do projeto com o robô atingindo o ponto desejado, clique [aqui](https://drive.google.com/file/d/1W6UDqgpirMCnvRqb7XizbO5vyU9BOG-L/view?usp=sharing).

### 📷 Screenshots

<p align="center">
  <img height=200px src="./docs/mapping/Screenshot (447).png" />
  <img height=200px src="./docs/mapping/Screenshot (449).png" />
  <img height=200px src="./docs/mapping/Screenshot (450).png" />
  <img height=200px src="./docs/mapping/mob_robM_2023_04_03__02_14_58.png" />
</p>

## Run Mapping Mode

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

### Video

Para acessar o vídeo da execução do projeto com o robô atingindo o ponto desejado, clique [aqui](https://drive.google.com/file/d/1W6UDqgpirMCnvRqb7XizbO5vyU9BOG-L/view?usp=sharing).

### 📷 Screenshots

<p align="center">
  <img height=200px src="./docs/mapping/Screenshot (447).png" />
  <img height=200px src="./docs/mapping/Screenshot (449).png" />
  <img height=200px src="./docs/mapping/Screenshot (450).png" />
  <img height=200px src="./docs/mapping/Screenshot (451).png" />
  <img height=200px src="./docs/mapping/Screenshot (452).png" />
  <img height=200px src="./docs/mapping/Screenshot (456).png" />
  <img height=200px src="./docs/mapping/Screenshot (458).png" />
  <img height=200px src="./docs/mapping/Screenshot (460).png" />
  <img height=200px src="./docs/mapping/Screenshot (461).png" />
  <img height=200px src="./docs/mapping/mob_robM_2023_04_03__02_14_58.png" />
</p>

## Run Autonomous Mode

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

### Vídeo da execução

Para acessar o vídeo da execução do projeto com o robô atingindo o ponto desejado, clique [aqui](https://drive.google.com/file/d/1W6UDqgpirMCnvRqb7XizbO5vyU9BOG-L/view?usp=sharing).

### 📷 Screenshots

<p align="center">
  <img height=200px src="./docs/mapping/Screenshot (447).png" />
  <img height=200px src="./docs/mapping/Screenshot (449).png" />
  <img height=200px src="./docs/mapping/Screenshot (450).png" />
  <img height=200px src="./docs/mapping/mob_robM_2023_04_03__02_14_58.png" />
</p>

## Execution

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


## References

https://ethz-robotx.github.io/SuperMegaBot/core-software/installation_core.html

https://ethz-robotx.github.io/SuperMegaBot/core-software/HowToRunSoftware.html

https://unlimited.ethz.ch/display/ROBOTX/SuperMegaBot

https://github.com/ethz-asl/eth_supermegabot

https://github.com/cra-ros-pkg/robot_localization/blob/noetic-devel/params/ekf_template.yaml