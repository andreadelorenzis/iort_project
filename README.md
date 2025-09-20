# IoRT Project - Internet of Robotic Things Vacuum System

This project was developed as a case study for the course **Programming for the Internet of Things** at the University of Urbino Carlo Bo.  
The goal is to integrate a low-cost commercial robotic vacuum cleaner into an **IoRT (Internet of Robotic Things)** ecosystem, enhanced with environmental sensors and an IoT infrastructure based on Docker containers.

Here's a short description of the main components of this project:
- **MQTTSensorNode**: Arduino (ESP32) code for MQTT sensor nodes
- **iotstack**: Backup of the IOTstack configuration (Docker + services)
- **vacuumbot**: ROS2 package for the robotic vacuum
- **web_interface**: Web interface for control and configuration


## VacuumBot (ROS2)

ROS2 package that handles:
- Integration with the physical vacuum cleaner via IR commands  
- Integration with the IORT system via a MQTT bridge


<p align="center">
  <img src="imgs/robot_top2.png" alt="VacuumBot ROS2" width="400"/>
</p>


## MQTT Sensor Node (ESP32)

Arduino sketch (`MQTTSensorNode.ino`) for an **ESP32** board.  
It reads data from multiple sensors:
- PIR (motion)  
- IR (obstacles)  
- Light  
- Temperature & Humidity  

Then it publishes data periodically to the MQTT broker.

## IoT Infrastructure (IOTstack)
Built with Docker containers:
- **Mosquitto** → MQTT broker  
- **Node-RED** → Flow management & automation  
- **InfluxDB** → Time-series database  
- **Grafana** → Visualization dashboards  
- **MariaDB** → Storage for rules and configs  
- **Nginx** → Serves the web interface  
- **Portainer** → Docker management  

## Other ROS packages used
The `vacuumbot/` package provides the main integration with the robotic vacuum cleaner, enabling both simulation and real-world operation.  
It works together with several ROS2 packages:

- **slam_toolbox** → Provides mapping and localization (SLAM) so the robot can build and use a map of the environment.  
- **nav2 (Navigation2)** → Handles path planning, obstacle avoidance, and autonomous navigation.  
- **turtlebot3_gazebo** (for simulation) → Used to simulate the robot in a Gazebo environment before testing on the real device.  
- **rviz2** → Visualization tool to monitor the robot’s sensors, map, and trajectory.  
- **teleop_twist_keyboard** → Allows manual robot control from the keyboard (useful for testing).  

**Coverage zone example**

<p align="center">
  <img src="imgs/coverage_path_example.png" alt="ROS2 Packages Overview" width="400"/>
</p>

## Web Interface

A simple HTML/CSS/JS interface for:
- Configuring cleaning zones  
- Defining rules and schedules  
- Sending commands to the robot  
- Monitoring sensors in real-time  

**Map control page**
<p align="center">
  <img src="imgs/web_interface_map.png" alt="Web Interface" width="400"/>
</p>

**Sessions control page**
<p align="center">
  <img src="imgs/test_page_screen.png" alt="Web Interface" width="400"/>
</p>

## Documentation
For more details on the achieved results, refer to the following: <a href="/presentation.pdf" target="_blank">presentation</a>.

Here you can read the <a href="report/report.pdf" target="_blank">full report</a> (in italian though). 

