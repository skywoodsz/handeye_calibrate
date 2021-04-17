## 1. 手眼标定原理

详见 *calibrate.pdf*

无论是eye_to_hand 还是 eye_in_hand 最后都可以化成AX=XB的形式，同时可利用标定板的摆放使得一些转换矩阵变为单位阵，从而化简标定。

如：在eye_to_hand中，可将标定板（board）坐标系与base坐标系对齐，从而使得 T_end2board * T_base2end = I，从而有 T_base2camera = T_board2amera.

## 2. 标定过程

1. aubo 等支持 moveIt 包的机械臂，可利用easy_hand进行标定，详见 [aubo_pick_up_with_6D](https://github.com/skywoodsz/aubo_pick_up_with_6D)

2. 不支持 moveIt 的机械臂标定

   ### 2.1 标定板识别： apriltag, 不建议 aruco （坐标混乱）

   apriltag 安装：

   ```
   export ROS_DISTRO=melodic               # Set this to your distro, e.g. kinetic or melodic
   source /opt/ros/$ROS_DISTRO/setup.bash  # Source your ROS distro 
   mkdir -p ~/catkin_ws/src                # Make a new workspace 
   cd ~/catkin_ws/src                      # Navigate to the source space
   git clone https://github.com/AprilRobotics/apriltag.git      # Clone Apriltag library
   git clone https://github.com/AprilRobotics/apriltag_ros.git  # Clone Apriltag ROS wrapper
   cd ~/catkin_ws                          # Navigate to the workspace
   rosdep install --from-paths src --ignore-src -r -y  # Install any missing packages
   catkin build    # Build all packages in the workspace (catkin_make_isolated will work also)
   ```

   ### 2.2  realsen 安装

   详见 [realsense](https://github.com/IntelRealSense/realsense-ros)

   *Notes：* 标定中，*camera_frame* 选择 *camera_color_optical_frame* ，即成像光轴，详情可见apriltag_ros launch 配置

   ### 2.3 数据获取

   在 *eye_in_hand* 中需要获取 *T_gripper2base T_target2cam*，从而获得 *T_cam2gripper*

   在 *eye_to_hand* 中需要获取 *T_base2gripper T_cam2target*，从而获得 *T_cam2gbase

   *Notes：*

    1. 在获取转换矩阵时最好使用 *tf* 指令查看转换关系，即：

   ```
   rosrun tf tf_echo [reference_frame] [target_frame]
   ```

   2. 注意齐次转换矩阵的逆中平移部分，并不是原平移向量的逆，而是
      $$
      T^{-1} = [R^{T}, -R^{T} * tR^{-1}; 0, 1]
      $$
      

###  2.4 数据处理

可以用程序 calibrate.cpp进行处理，注意上述中不同形式下所用数据间的区别即可

> 处理程序参考：[opencv手眼标定](https://blog.csdn.net/weixin_42203839/article/details/103882739)

*Notes：* 一些四元数在线转换网站：[3D可视化](https://www.andre-gaschler.com/rotationconverter/)