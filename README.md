bringup内为Launch启动文件
doc内为各功能包实现
  - abb_ros2功能包[^1]实现了机械臂与Moveit2的通信,对应abb_control与fabrication_sensor_cpp两个launch文件;
  - vision_calibration功能包[^2]通过视觉实现了全工作空间的坐标系统一,对应cameraToobject_calibration与yolov8_3d以及realsense三个launch文件;
  - path_generation功能包[^3]给出了从平面到三维曲面的图案路径投影变换,参考tests示范文件使用;
  - mcu_ros2功能包[^4]给出了将STM32/Arduino等单片机连接到ROS2通信网络中的实现方案;
    
[20240611功能包仍在开发中,本项目仅作个人学习记录使用]

bringup is the Launch startup file.
Inside the doc is the implementation of the source packages:
  - abb_ros2 package[^1] implements the communication between physical ABB IRB1600 and Moveit2, corresponding to abb_control and fabrication_sensor_cpp launch file; 
  - vision_calibration package[^2] realises the coordinate system unification in the whole workspace through vision, corresponding to cameraToobject_cpp launch file; 
  - path_generation package[^3] gives a patterned path projection transformation from planar to 3D surfaces, refer to the tests file; 
  - mcu_ros2 package[^4] gives an implementation of connecting STM32/Arduino microcontrollers to the ROS2 communication network; 

[20240611 source packages are still under development. This project is only for personal record].

[^1]: https://github.com/PickNikRobotics/abb_ros2
[^2]: https://intel.github.io/ros2_grasp_library/docs/doc/overview.html
[^3]: https://coursehome.zhihuishu.com/courseHome/1000097785
[^4]: https://micro.ros.org/docs/tutorials/core/overview/




