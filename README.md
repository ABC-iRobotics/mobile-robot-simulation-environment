mobile-robot-simulation-environment

## Experimental Implementation of Obstacle Avoidance Algorithms for Autonomous Agricultural Vehicle in a Simulation Environment

## Table of Contents
1. [Summary](#1-summary)
2. [Developed Algorithm](#2-developed-algorithm)
   - [2.1. Navigation and Control Algorithm](#21-navigation-and-control-algorithm)
   - [2.2. Sensor Data Processing](#22-sensor-data-processing)
3. [Installation and Development Steps](#3-installation-and-development-steps)
   - [3.1. Required Tools and Software](#31-required-tools-and-software)
   - [3.2. Installing WSL and Ubuntu](#32-installing-wsl-and-ubuntu)
     - [3.2.1. Installing WSL](#321-installing-wsl)
     - [3.2.2. Installing Ubuntu](#322-installing-ubuntu)
   - [3.3. Installing Webots](#33-installing-webots)
   - [3.4. Installing ArduPilot](#34-installing-ardupilot)
   - [3.5. Installing and Configuring MAVProxy](#35-installing-and-configuring-mavproxy)
4. [Deployment Guide](#4-deployment-guide)
   - [4.1. Starting the ArduPilot Simulation](#41-starting-the-ardupilot-simulation)

## 1. Summary

The goal of the project is to develop an algorithm that enables the robot to efficiently and safely detect and avoid obstacles while in motion. The robot's control and navigation are provided by the already implemented ArduPilot solution, so the algorithm being developed must integrate into its operation. Due to limited access to the physical robot, it is necessary to create a 3D simulation environment in which the algorithm can be developed and tested. For this, the simulation environment must be able to work in conjunction with the ArduPilot simulation as well.

## 2. Developed Algorithm

### 2.1. Navigation and Control Algorithm
The goal of the project is to enable the robot to navigate between two points (A and B) in a simulated space. The control and navigation are based on the following algorithms:

- **ArduPilot Autopilot System**: The system responsible for controlling the robot, which is capable of following predefined paths (waypoints), avoiding obstacles, and ensuring stable navigation.
- **Sensor Data Processing**: The robot is equipped with various sensors (e.g., cameras and RangeFinders) that provide information about the environment. By processing the sensor data, the robot can modify its path and mission to avoid obstacles.

To improve the precision of navigation and control, an algorithm are applied during the project. Continuous analysis of sensor data (e.g., from RangeFinders) to ensure the robot avoids obstacles in its environment.

### 2.2. Sensor Data Processing
The robot’s navigation and localization are supported by various sensors:
- **Camera Sensors**: These are used in the Webots simulation environment to model the robot's vision.
- **RangeFinder Sensors**: These sensors measure the distance to the robot's immediate surroundings, which are crucial for obstacle avoidance and spatial navigation.

## 3. Deployment Steps

### 3.1. Required Tools and Software

The following tools and software are required for the project:

- **WSL (Windows Subsystem for Linux)**: Allows running Linux-based tools on Windows.
- **Ubuntu Operating System**: The Linux distribution used inside WSL for development and execution.
- **Webots**: A simulation environment for robot modeling and testing.
- **ArduPilot**: An open-source autopilot system for controlling robots.
- **MavProxy**: An add-on for ArduPilot wich helps with the communication outside of ArduPilot
- **QGroundControl**: An open-source add-on for ArduPilot, with better UI and functionaliy

### 3.2. Installing WSL and Ubuntu

#### 3.2.1. Installing WSL

The Windows Subsystem for Linux (WSL) installation allows you to run a Linux distribution on Windows. Follow these steps:

1. **Enable WSL**:
   - Open PowerShell as Administrator and run the following command:

     ```bash
     wsl --install
     ```

2. **Install Ubuntu**:
   - Open the Microsoft Store, search for "Ubuntu", and choose the desired version (e.g., "Ubuntu 20.04 LTS").
   - Click the "Install" button.

3. **Verify WSL Version**:
   - After installing WSL, check that the system is working correctly:

     ```bash
     wsl --list --verbose
     ```

#### 3.2.2. Installing Ubuntu

1. **Update the system’s package list**:

    ```bash
    sudo apt update
    ```

2. **Install the required packages**:

    ```bash
    sudo apt install git g++ make python3 python3-pip python3-dev python3-setuptools
    ```

### 3.3. Installing Webots

1. **Download Webots**:
   - Download Webots from the official website: [Webots Download](https://cyberbotics.com/)
   - Follow the installation guide for Ubuntu.

2. **Setting up Webots and Creating Simulation Projects**:
   - Create or open the Webots project that includes a robot with the RangeFinder sensor equipped.

### 3.4. Installing ArduPilot

1. **Clone ArduPilot from GitHub**:

    ```bash
    git clone https://github.com/ArduPilot/ardupilot.git
    cd ardupilot
    ```

2. **Initialize Submodules**:

    ```bash
    git submodule update --init --recursive
    ```

3. **Set up the environment**:

    ```bash
    ./Tools/environment_install/install-prereqs-ubuntu.sh -y
    ```

4. **Build the simulation environment**:

    ```bash
    ./waf configure
    ./waf build
    ```

### 3.5. Installing and Configuring MAVProxy

1. **Install MAVProxy**:

    ```bash
    sudo apt install python3-pip
    pip3 install MAVProxy
    ```

2. **Start MAVProxy**:

    ```bash
    mavproxy.py --master=udp:127.0.0.1:14550 --console --map
    ```

## 4. Starting the simulation
### 4.1 Starting ArduPilot 

To start the simulation, use the following command:

```bash
sim_vehicle.py -v Rover webots-python -w --speedup=1 --console --speedup=1 --map
```
![image](https://github.com/user-attachments/assets/1629577d-cabc-4c25-a63a-16bd2f0374f7)

### 4.1 Starting Webots 

Start the simulation with the play button on the action bar
![image](https://github.com/user-attachments/assets/4f50c095-d924-4568-afa2-fc2068b5db03)


