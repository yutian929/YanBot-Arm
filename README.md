# YanBot-Arm

ARX robotic arm ROS Noetic version, designed for ALOHA desktop, featuring dual arm teaching and dual arm execution capabilities. This package includes CAN communication setup nodes, CAN communication startup nodes, master-slave control nodes, and more.

## Table of Contents

- [Prerequisites](#prerequisites)
- [1. Prepare & Install](#1-prepare--install)
- [2. Setup CAN Communication](#2-setup-can-communication)
- [3. Usage](#3-usage)
  - [3.1 Master-Slave Control (Teach and Follow)](#31-master-slave-control-teach-and-follow)
  - [3.2 RGB-D Camera Testing](#32-rgb-d-camera-testing-including-d405)
- [4. Additional Information](#4-additional-information)
- [5. License](#5-license)
- [6. Contributing](#6-contributing)

## Prerequisites

Before installing YanBot-Arm, ensure that you have the following prerequisites installed on your system:

- **Ubuntu 20.04 (Focal Fossa)**
- **ROS Noetic**
- **Git**
- **CAN Bus Hardware and Drivers**
- **LibRealsense**

Ensure your ROS environment is properly set up by following the [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu).

Or you can execute the following command directly to achieve one-click installation
```bash
wget http://fishros.com/install -O fishros && . fishros
```


Ensure your Realsense-ros environment is set up by following the [librealsense 2.50.0 realsense-ros 2.3.2](https://gitee.com/linClubs/librealsense-2.50.0) till STEP2.
```bash
# install librealsense-2.50.0
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev 

git clone https://gitee.com/linClubs/librealsense-2.50.0.git
cd librealsense-2.50.0
./scripts/setup_udev_rules.sh

mkdir build
cd build
cmake ..
make -j4
sudo make install
# test
realsense_viewer
```

## 1. Prepare & Install

Follow these steps to prepare and install the YanBot-Arm:

1. **Clone the Repository**

    ```bash
    cd ~
    git clone https://github.com/yutian929/YanBot-Arm.git
    ```

2. **Navigate to the Workspace**

    ```bash
    cd ~/YanBot-Arm  # ROS Noetic workspace
    ```

3. **Install Dependencies**

    Ensure you have the necessary dependencies by running:

    ```bash
    bash install_deps.sh
    ```

4. **Build the Package**

    Compile the package using `catkin_make`:

    ```bash
    catkin_make
    ```

    After successful compilation, source the workspace:

    ```bash
    source devel/setup.bash
    ```

## 2. Setup CAN Communication

Setting up CAN communication is crucial for the master-slave functionality of the dual arms. Follow these steps:

1. **Source the Workspace Setup File**

    ```bash
    source ~/YanBot-Arm/devel/setup.bash
    ```

2. **Run the CAN Setup Node**

    ```bash
    rosrun arx_pkg setup_can_node.py
    ```

3. **Connect CAN Devices**

    - **Disconnect All CAN Communication Devices:** Ensure that all CAN devices are disconnected before starting the setup.
    - **Sequentially Connect Devices:** Follow the on-screen instructions to connect the devices in the following order:
        1. Master Left
        2. Follower Left
        3. Master Right
        4. Follower Right

    **Note:** This setup process only needs to be performed once.

## 3. Usage

### 3.1 Master-Slave Control (Teach and Follow)

To operate the ARX robotic arms in a master-slave configuration for teaching and following movements, follow these steps:

1. **Start CAN Communication**

    Open three separate terminal windows for the following commands:

    - **Terminal 1:** Start `roscore`

        ```bash
        roscore
        ```

    - **Terminal 2:** Start the CAN communication node

        ```bash
        rosrun arx_pkg start_can_node.py
        ```

2. **Launch Master and Follower Nodes**

    - **Terminal 3:** Launch the `master_and_follower.launch` file

        ```bash
        roslaunch arx_pkg master_and_follower.launch
        ```

    This will initialize the master and follower arms. The follower arms will replicate the masters' movements in real-time.

### 3.2 RGB-D Camera Testing (Including d405)
1. **Launch rgbd.launch**

    ```bash
    roslaunch realsense2_camera rs_rgbd.launch 
    ```
    
    The wrapper of relasense-ros in our code has been updated, which means it can support d405 camera in ROS1.  

## 4. Additional Information

- **Reference Repository:** 
  - [ARX_R5 GitHub repository](https://github.com/ARXroboticsX/ARX_R5).

  - [librealsense 2.50.0 realsense-ros 2.3.2(ros1) Gitee repository](https://gitee.com/linClubs/librealsense-2.50.0).

  - [realsense-ros Github repository](https://github.com/IntelRealSense/realsense-ros).

- **ROS Environment Setup:** Ensure that your ROS Noetic environment is correctly configured. You can refer to the [ROS Wiki](http://wiki.ros.org/noetic) for comprehensive setup guides.

- **Troubleshooting:** If you encounter issues during installation or usage, consult the package documentation or open an issue on the [GitHub repository](https://github.com/ARXroboticsX/ARX_PKG/issues).

## 5. License

Not sure yet.

## 6. Contributing

Contributions are welcome! If you would like to contribute to YanBot-Arm, please follow these steps:

1. Fork the repository.
2. Create a new feature branch.
3. Commit your changes.
4. Push to the branch.
5. Open a pull request detailing your changes.

Please ensure that your contributions adhere to the project's coding standards and include appropriate documentation.