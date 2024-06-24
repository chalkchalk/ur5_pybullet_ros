# UR5-mobile Task Demo (Navigation and Grasping)
## Features
- Auto Navigation for mobile grasping task
- Grasping behavior of the robot arm
- A task demo showing the capability of the mobile arm system.
## Install
### Clone the repo
```sh
git clone <code repo url>
```
The path of the folder `code folder path` that contains the `src` folder in the repo will be used to mount directory when creating container.
### Docker 
In order to use the image, you need to install docker and docker nvidia-runtime (if you have an Nvidia GPU) first.

Reference: [Ubuntu18.04安装NVIDIA Docker 2](https://blog.csdn.net/weixin_38369492/article/details/105809571)

If you don't have an Nvidia GPU, then just install docker only.
### Docker image

Use the following command to pull the docker image
```sh
docker pull chalkchalk/ur5_pybullet_moveit:20240130
```

Use `docker ps -a` to check if there is any previous version of the container. If there is, use `docker rm <contain-name>` to delete it.

Then, create the container with the new version of the image:
- If you have an Nvidia GPU with nvidia-runtime installed:

```sh
docker run -dit\
    --name ur5_pybullet_moveit \
    --cap-add=SYS_PTRACE \
    --gpus all \
    --shm-size=4G \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v <code folder path>/src:/root/catkin_ws/src \
    -e DISPLAY=unix$DISPLAY \
    -e NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} \
    -e NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics \
    --privileged \
    --network=host\
    chalkchalk/ur5_pybullet_moveit:20240130
```
- If you don't:
```
docker run -dit\
    --name ur5_pybullet_moveit \
    --cap-add=SYS_PTRACE \
    --shm-size=8G \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v <code folder path>/src:/root/catkin_ws/src \
    -e DISPLAY=unix$DISPLAY \
    --privileged \
    --network=host\
    chalkchalk/ur5_pybullet_moveit:20240130
```
NOTE: you only need to run `docker run` to create the container for ONCE. After that, you only need to START the container each time you use it.

## Run the Code
### Start the container

In order to enable the GUI in the container, remember to run
```
xhost +
```
in your terminal each time you log in your system. Otherwise, GUI like Rviz or pybullet will not be shown in docker container.

It is recommended to add this code to startup applications by running `gnome-session-properties` in your gnome terminal.

Start the docker container:

 ```sh
docker start ur5_pybullet_moveit
 ```
Then, attach to the shell in the container:

 ```sh
docker exec -it ur5_pybullet_moveit bash
 ```

### Build
NOTE: All the commands below are supposed to run in the terminal of the container, instead of your host machine.
Build the code.
```sh
cd /root/catkin_ws/
catkin build
```
Then, source the repo:
```sh
source /root/catkin_ws/devel/setup.bash
```
NOTE: You only need to run the commands above ONCE. After you finish setting them up when creating the container, just simply run the code next time you boot the container.

### Run
#### Pybullet with ROS and Moveit simulation
Use the following command to launch the simulation environment of the mobile UR5 with Moveit robot arm controller.
```sh
roslaunch ur5_pybullet_ros ur5_pybullet_moveit.launch
```
Then, the simulation environment is launched, including a pybullet GUI and an Rviz window. 
You can click  `2D Nav Goal` in rviz panel, and click on the map to select a navigation target for the robot. Then, the robot will navigate to the given position, while constructing map. You can also drag the robot arm target pose to set the goal pose of the robot arm, then click `Plan & Execute` to control the robot arm.

#### Task Demo

After you launch Pybullet with ROS and Moveit simulation, Please open another terminal and attach to the bash shell of the container, use the following command to launch the task controller for the demo demonstration.
```sh
rosrun ur5_pybullet_ros task_controller.py
```
Then, the task controller will automatically send navigation and arm commands to the robot, controlling it to finish the demo task. The robot will navigate to the desk, pick up the red ball, and drop it into the tray located in the other side of the room. A dynamic obstacle is also added to show the capability of the robot to navigate through dynamic environment. Then, the robot will pick up the green ball, and put it back into the other tray near where it started.


