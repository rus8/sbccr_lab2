# ecn_sensorbased
Sensor-based control lab 2 at École Centrale de Nantes.

## Prepare environment
Instructions are provided for Ubuntu 16 and ROS Kinetic.

1. Create workspace: <br>
    ```
       mkdir sbccr_ws
       cd sbccr_ws
       mkdir src
       cd src
    ```

2. Clone this repo in workspace src/ folder: <br>
    ```git clone https://github.com/rus8/sbccr_lab2.git```
3. Clone ECN common package in workspace src/ folder: <br>
    ```git clone https://github.com/oKermorgant/ecn_common```
4. VISP library: <br>
    ```sudo apt-get install ros-kinetic-visp```
5. VREP library has to be installed via bash script ([description](http://pagesperso.ls2n.fr/~kermorgant-o/coding%20tools.html#scripts)). Download the [script](https://gitlab.univ-nantes.fr/kermorgant-o/ecn_install/raw/master/vrep_system_install.sh) and run it: <br>
    ```bash vrep_system_install.sh``` <br>
Then follow the instructions provided in your terminal. According to them it's required to clone `vrep_ros_launcher` package in workspace src/ : <br> 
    ``` git clone https://github.com/oKermorgant/vrep_ros_launcher.git```
6. Build your workspace. Run in workspace root folder: <br>
    ``` catkin build # using catkin_tools```


## Run the lab

1. Being in your workspace root folder source environment: <br>
    ```source devel/setup.bash```
2. Launch VREP: <br>
    ```roslaunch ecn_sensorbased vrep.launch```
3. Run control node: <br>
    ```rosrun ecn_sensorbased pioneer```
