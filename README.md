# RobotTrajectoryAnalyzer



## Dependencies
* [UIMA C++ 2.4.0](http://uima.apache.org/) - set the UIMACPP_HOME environment variable!
* [MongoDB C++ Legacy Driver](https://github.com/mongodb/mongo-cxx-driver), 1.0.3
* [rqt_plot](http://wiki.ros.org/rqt_plot)
* [PyQtGraph](http://www.pyqtgraph.org/) *(optional)*



## Compile & run

```shell
$ catkin_make
$ source ./devel/setup.bash
```

### Running the standalone annotation pipeline

```shell
$ ./devel/lib/rta_annotators/demo
```

### Using `rqt_plot` for visualizations

![Accelerations visualized using rqt_plot with PyQtGraph](https://cloud.githubusercontent.com/assets/3015996/8634113/a42f4286-27eb-11e5-94ce-5ad71d107bf0.png)

First get the topic information you want to visualize, use `TAB` for listing all availbale `rta_nodes`.

```shell
$ rosrun rta_nodes acceleration_topic -loop
$ rostopic list
/rosout
/rosout_agg
/rtaAcceleration
$ rostopic echo /rtaAcceleration
layout:
  dim:
    -
      label: accelerations
      size: 44
      stride: 1
  data_offset: 0
data: [0.0, ...]
```

The initialize the visualization using the topic name + data array length and restart the topic again:

```shell
$ ./scripts/plottopic.sh rtaAcceleration 44
$ rosrun rta_nodes acceleration_topic
```

`rqt_plot` is most useful if you use [PyQtGraph](http://www.pyqtgraph.org/) and its auto scaling functionality (bottom left of the plot).
