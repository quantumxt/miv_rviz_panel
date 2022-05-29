# Multi-Image View Rviz Plugin

A multi-image gridview panel widget to display various image feed in a 2x2 grid.

> Currently support only rgb8 `sensor_msgs/Image/` type.

![](assets/miv_main.png)

## Installation

Clone the repository into the `src` directory of your current workspace.

```bash
git clone https://github.com/1487quantum/miv_rviz_plugin.git
```

After that, compile and re-source the workspace.

```bash
cd ~/catkin_ws
catkin build miv_rviz_plugin
source ~/catkin_ws/devel/setup.bash
```

The plugin should be available to be added into Rviz. (Through Panels > Add New Panel)

![](assets/rviz_panel.png)
