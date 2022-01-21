# jetleg_description

URDF models, meshes, & visualization tools for JetLeg

![jetleg_description rviz](https://user-images.githubusercontent.com/59701038/150576852-7605f6a8-f3eb-46a4-8296-c8e3ff1d2807.png)

## Launch visualization

```
ros2 launch jetleg_description visualize_urdf.launch.py model:=`ros2 pkg prefix --share jetleg_description`/urdf/testrig.xacro
```

## URDF/xacro

Please refer to [ROS2 URDF Tutorials](https://docs.ros.org/en/foxy/Tutorials/URDF/URDF-Main.html) for more info about urdf modeling and xacro
