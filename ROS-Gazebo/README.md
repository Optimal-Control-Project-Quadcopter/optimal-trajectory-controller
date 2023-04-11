# Testing in ROS-Gazebo Simulator

## About

- The MAV model & dynamics are implemented in [RotorS](https://github.com/gsilano/CrazyS) framework, based on which a quadcopter was derived for an national Competition-cum-MOOC by [e-Yantra](https://www.e-yantra.org/) to teach control-theory & other programming stack to provide a taste of real feedback systems on an RTOS application.

	- RotorS package 

	- Sentinel Drone package

> Contact @vishalgpt579 for more detail about ROS's implementation of this project.


## Setup

- A Ubuntu-20 docker container was implement the ...
	- Cloning some of Github package
	- Installing necessary library for proper file build.
	- Structural safety of the host computer, since any build-fail within the docker won't effect the host.

### Installing Docker

- Follow the step mentioned on the [Nvidia-Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installation-guide)installation guide to get the basic tools needed for docker on your system.

- **IMPORTANT**: Run the command mentioned below in your terminal to create a container with preinstalled screen and graphic drivers. 

```bash
docker run -it     --env="DISPLAY=$DISPLAY"     --env="QT_X11_NO_MITSHM=1"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"     --env="XAUTHORITY=$XAUTH"     --volume="$XAUTH:$XAUTH"     --net=host     --privileged     --gpus all     nvidia/cuda:11.0.3-devel-ubuntu20.04
``` 
#### Test GUI

Once the container is ready, run the following command to test if GUI works.

```bash
apt update
```

> NOTE: if it is the first time opening the container, one might need to run `apt install sudo build-essentials` before proceeding further.

```bash
apt install mesa-utils

glxgears
```

### Installing Simulation Packages

> The commands mentioned below assumes that one already has **ROS-noetic**  installed & a workspace ready. If not, please follow the tutorial [here](http://wiki.ros.org/noetic/Installation/Ubuntu)

- Move to your workspace and then move forward with the next steps.
```bash
cd ~/catkin_ws/src
``` 

### ROS Packages Requirements

- `sudo apt install ros-noetic-mavros-dbgsym ros-noetic-mavros-msgs`

- `sudo apt install ros-noetic-octomap*`

- `sudo apt install python2`

> Python isn't installed by-default in this container. Install `python3` and `python2` separately. However, installing `pip2` is troublesome, one can do that using the following commands. 
> - `curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py`
> - `python get-pip.py`

- `python -m pip install catkin_pkg`

- `python -m pip install pyyaml`

- `python -m pip install rospkg`

```bash
sudo apt install libgoogle-glog-dev libgflags-dev protobuf-compiler libprotobuf-dev

sudo apt-get install ros-noetic-mavros-msgs ros-noetic-mavlink
```

### MAV Packages

Install all the package with your `src` folder.

```bash
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/mav_comm.git
git clone https://github.com/ethz-asl/eigen_catkin.git
git clone https://github.com/ethz-asl/mav_control_rw.git
git clone https://github.com/ethz-asl/rotors_simulator.git
git clone https://github.com/ethz-asl/mav_comm.git
git clone --recursive https://github.com/erts-RnD/sentinel_drone
```

### BUILD !!!

- Compile your workspace using `catkin_make` or `catkin build`. Make sure that the terminal's current directory is `~/catkin_ws`. Basically...

```bash
cd ~/catkin_ws
catkin_make
```
- Source the compiled packages, to make the system aware of their existence.
```bash
source ~/catkin_ws/devel/setup.bash
```
- Once built, run `python ~/catkin_ws/src/rotors_simulator/rqt_rotors/setup.py`, to install dependencies required for `hil_plugin`.
	- The above line will give an error, which can be removed using ...
	```bash
	python ~/catkin_ws/src/rotors_simulator/rqt_rotors/setup.py install
	```
## Test

- Launch the simulated world, containing ...
	1. A quadcopter with a "Whycon" marker for localisation.
	2. Downward facing camera, acting as a satellite.

```bash
roslaunch sentinel_drone task2.launch
```

- Run the executable file `position_hold` to see a quadcopter hovering at a set pose. Download it from this [drive link](https://drive.google.com/file/d/1qP-poH6oHZ9fj7J9EjRxN9f-tweiRpnK/view?usp=share_link)
```bash
chmod +x position_hold
./position_hold
``` 

- One can see the drone taking off and holding it position. Try giving an impulse (disturbance) to see the PID loop, making it come back to it original location.