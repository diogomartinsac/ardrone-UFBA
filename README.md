# Projeto Final Mat701

Projeto Final da disciplina de MAT701. Tópicos Especiais em Mecatrônica IV - Programaçao para Drones (2019.2) 
Prof. Alirio Sá

**Pré-requisitos** 


1.  Precisa instalar o Ubutun na versão 16 


2.  Precisar instalar o Kinect 

3. Precisar instalar o gazebo

4. Precisar ter instalado o Python na versão 2.7 (sim caso tiver com o anaconda ou Python 3 em diante é definir o python 2 como padrão no ubutun, veja na net sobre o assunto essa discursão dará um norte para resolver o problema https://stackoverflow.com/questions/24405561/how-to-install-2-anacondas-python-2-and-3-on-mac-os) 

5. Precisar instalar o hector e suas dependências. Comando:  sudo apt-get install ros-kinetic-hector-* 

6. Precisar instalar o ardrone_autonomy e suas dependencias . Comando: sudo apt-get install ros-kinetic-ardrone-autonomy 

7. Precisar instalar o tum_simulator adaptado para rodar com o kinetct no git. Este item em especifico já está instalado na aplicação e já foi feito o upload do mesmo no projeto, a aplicação foi baixada no https://github.com/angelsantamaria/tum_simulator. 

 

**Rodando a aplicação** 

1. Em cada terminal aberto rode o comando sudo para o mesmo ter acesso ao setup da aplicação. Na minha máquina o eu uso o seguinte comando: source  ~/estudos_ros/pj_drone_ws/devel/setup.bash  

2. Compile o projeto (catkin_make) 

3. Simule o drone com o gazebo. Comando: roslaunch cvg_sim_gazebo ardrone_testworld.launch  

4. Testar para o drone decolar rostopic pub -1 ardrone/takeoff std_msgs/Empty ou rosrun drone_custom decolar.py (veja mais aqui https://edu.gaitech.hk/drones/ar_parrot_2/ar-parrot-2-ros.html#control-the-robot-using-tum-ardrone-package) 

**Atenção** 

1. Todos os pacotes de códigos customizado(python ou c++) estou devem ser colocados no pacote drone_custom 

2. Avisar em quais funcionalidades estão sendo trabalhadas para não trabalhar na mesma coisa 

