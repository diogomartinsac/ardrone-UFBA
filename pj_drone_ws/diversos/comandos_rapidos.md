### entrar na pasta padrão
cd entregaveis/pj_drone_ws/
### dar direitos ao projetos
source ~/PycharmProjects/mat701-20192/entregaveis/pj_drone_ws/devel/setup.bash
### ativar o conda
conda activate python2

###rodar o simulador
roslaunch cvg_sim_gazebo ardrone_testworld.launch

### parameratizar controle de game
rosparam set joy_node/dev "/dev/input/js1"


terminal command control
take off (decolar)
rostopic pub -1 /ardrone/takeoff std_msgs/Empty

land 
rostopic pub -1 /ardrone/land std_msgs/Empty
switch camera

rosservice call /ardrone/togglecam
motion

# fly forward
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

# fly backward
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -1.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

# fly to left 
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

# fly to right 
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -1.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

# fly up 
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

# fly down 
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: -1.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

# counterclockwise rotation
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 1.0}}'

# clockwise rotation
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: -1.0}}'

# stop
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
Robot Sensor Monitoring
For observation you can use following commands:


# The output camera
rosrun image_view image_view image:=/ardrone/image_raw

# The front camera
rosrun image_view image_view image:=/ardrone/front/image_raw

# The buttom camera
rosrun image_view image_view image:=/ardrone/bottom/image_raw

# The height sensor
rostopic echo /sonar_height

#The navigation info
rostopic echo /ardrone/navdata

#A launch file for joystick drivers and image view nodes
roslaunch cvg_sim_test demo_tool.launch

#matar o roscore
killall -9 roscore
killall -9 rosmaster

#diminuir tamanho do endereco do linux no bash
https://askubuntu.com/questions/145618/how-can-i-shorten-my-command-line-bash-prompt
PROMPT_DIRTRIM=1

#catkin_com debug
catkin_make -DCMAKE_BUILD_TYPE=Debug