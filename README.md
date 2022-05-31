# Multi-Image View Rviz Plugin

A multi-image gridview panel widget to display various image feed in a 2x2 grid.

> Currently support most 8bit image type for  **sensor_msgs/Image/** message. ( `rgb8`, `rgba8`, `mono8` & `bgr8`)

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


## License
<a href="LICENSE" ><img src="https://img.shields.io/github/license/1487quantum/miv_rviz_panel?style=flat-square"/></a>
