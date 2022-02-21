# jetleg
Software for the Pack Bionics JetLeg

## Summary of packages

- `jetleg_bringup` Launch files for starting up relevant tasks
- `jetleg_control` JetLeg teleoperation, sensing, planning, and actuation
- `jetleg_description` URDF files for simulations
- `jetleg_vision` Vision processing nodes


## Dependencies

- robot_state_publisher: 
```
sudo apt-get install ros-foxy-robot-state-publisher
sudo apt-get install ros-foxy-robot-state-publisher-gui
```
- xterm (for teleop):
```
sudo apt-get install -y xterm
```
