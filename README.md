<h1>iiti_soc_projects</h1>

This ros2 repo contains the files for IITI SoC Project (Simulation Based Control System Design for Mini Drone). The drone get position feed from an over head waycon camera. The position data is filtered with a simple moving-average filter. For a stable flight PID control for each axis will be used. Right now PID support altitude control only.

<h2>Reqiurement</h2>

- Ubuntu 22.04 LTS
- Ros2 Humble
- Gazebo Fortress

<h2>Installation Instructions</h2>

```
mkdir ~/drone_ws
cd ~/drone_ws
git clone --recurse-submodules https://github.com/Tse-Choes/iiti_soc_projects.git src
```
```
colcon build
source install/setup.bash
```

- Set IGN_GAZEBO_RESOURCE_PATH to include swift_pico_description/models

  ```
  export IGN_GAZEBO_RESOURCE_PATH=~/drone_ws/install/swift_pico_description/share/swift_pico_description/models/
  ```

- Launch the simulation with

  ```
  ros2 launch swift_pico swift_pico.launch.py
  ```

- To launch PID tunner use the following command.

  ```
  ros2 launch pid_tune pid_tune_drone.launch.py
  ```
