# PX4 Autopilot SITL Simulation Startup

To run a **PX4 Autopilot SITL (Software In The Loop)** simulation, you must first install **ROS 2 Foxy**, **PX4 Autopilot**, and all required dependencies.

Please follow the official PX4 documentation available at:

- [ROS 2 User Guide | PX4 Guide (main)](https://docs.px4.io/main/en/ros2/user_guide#foxy)

Make sure to complete the installation of:

- ROS 2 Foxy
- PX4 Autopilot
- Micro XRCE-DDS Agent
- All PX4 required dependencies
- ROS 2 workspace and related packages

## Starting the SITL Simulation

First, start the Micro XRCE-DDS Agent:

```bash
MicroXRCEAgent udp4 -p 8888
```

Open a second terminal and navigate to the PX4 source directory.

### Single Vehicle Simulation

```bash
cd PX4-Autopilot/
PX4_NO_FOLLOW_MODE=1 make px4_sitl gazebo
```

### Multi-Vehicle / Namespaced Simulation

```bash
cd PX4-Autopilot/
PX4_NO_FOLLOW_MODE=1 PX4_UXRCE_DDS_NS=px4_1 make px4_sitl gazebo
```

The `PX4_UXRCE_DDS_NS` environment variable assigns a DDS namespace to the vehicle and is useful when running multiple PX4 instances simultaneously.

## Verifying the Startup

If the startup is successful, you should see:

- The Micro XRCE-DDS Agent running and listening on UDP port `8888`
- The PX4 terminal displaying firmware status messages
- A Gazebo window showing the simulated vehicle

Once all components are running correctly, the system is ready for use with ROS 2 nodes and external control or monitoring applications.

# Rosbridge Installation and Startup

## Installing Rosbridge

Rosbridge requires a working ROS installation.

If ROS 2 Foxy has been installed following the PX4 guide above, install the rosbridge packages with:

```bash
sudo apt-get install ros-<rosdistro>-rosbridge-suite
```

This installs the complete rosbridge suite required to expose ROS topics and services through a WebSocket interface.

## Configuring the Environment

Before launching rosbridge, source the ROS environment:

```bash
source /opt/ros/<rosdistro>/setup.bash
```

## Starting Rosbridge

Navigate to the offboard control workspace and source the local setup script:

```bash
cd ws_offboard_control/
source install/local_setup.bash
```

Then start the rosbridge WebSocket server:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

By default, rosbridge will expose a WebSocket endpoint on port `9090`, allowing external applications to communicate with ROS 2 through the Rosbridge protocol.


## Starting the Drone Digital Twin

To start the Drone Digital Twin, just run the **DroneDigitalTwin** class located in /base directory. The Drone Digital Twin was implemented using [WLDT](https://wldt.github.io/) library:

```bibtex
@article{PICONE2021100661,
    title = {WLDT: A general purpose library to build IoT digital twins},
    journal = {SoftwareX},
    volume = {13},
    pages = {100661},
    year = {2021},
    issn = {2352-7110},
    doi = {https://doi.org/10.1016/j.softx.2021.100661},
    url = {https://www.sciencedirect.com/science/article/pii/S2352711021000066},
    author = {Marco Picone and Marco Mamei and Franco Zambonelli},
    keywords = {Internet of Things, Digital twin, Library, Software agent}
}
```
```bibtex
@INPROCEEDINGS{PICONE2025DCOSS,
  author={Picone, Marco and Martinelli, Matteo and Burattini, Samuele and Giulianelli, Andrea and Ricci, Alessandro},
  booktitle={2025 21st International Conference on Distributed Computing in Smart Systems and the Internet of Things (DCOSS-IoT)}, 
  title={The Two Faces of Interoperability: Bridging Cyber and Physical Spaces with Digital Twins}, 
  year={2025},
  volume={},
  number={},
  pages={1-8},
  keywords={Adaptation models;Technological innovation;Protocols;Software architecture;Soft sensors;Semantics;Smart systems;Digital twins;Interoperability;Standards;Digital Twins;Industrial Internet of Things;Cyber-Physical Systems;Interoperability},
  doi={10.1109/DCOSS-IoT65416.2025.00078}}
```

The Drone DT has the following Adapters:
* One Physical Adapter, called Rosbridge Physical Adapter, used to comunicate with the simulated drone.
* One HTTP DIgital Adapter, used to receive external commands sent with HTTP protocol and expose properties and events.
* One MQTT Digital Adapter, used to receive external commands sent with MQTT protocol and publish properties and events.

# Available Commands

## Arm the Vehicle

To arm the drone, send an HTTP POST request to the following endpoint:

```http
POST http://localhost:3001/state/actions/arm
```

Example using curl:

```bash
curl -X POST http://localhost:3001/state/actions/arm
```

## Take off

To take off the drone, send an HTTP POST request to the following endpoint:

```http
POST http://localhost:3001/state/actions/takeoff
```

Example using curl:

```bash
curl -X POST http://localhost:3001/state/actions/takeoff
```

## Land

To land the drone, send an HTTP POST request to the following endpoint:

```http
POST http://localhost:3001/state/actions/land
```

Example using curl:

```bash
curl -X POST http://localhost:3001/state/actions/land
```