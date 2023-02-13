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
