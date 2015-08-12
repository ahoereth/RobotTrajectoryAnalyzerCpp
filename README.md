# RobotTrajectoryAnalyzer



## Dependencies
* [UIMA C++ 2.4.0](http://uima.apache.org/) - set the UIMACPP_HOME environment variable!
* [MongoDB C++ Legacy Driver](https://github.com/mongodb/mongo-cxx-driver), 1.0.3
* [ROSCPP Indigo](http://wiki.ros.org/roscpp)
* [rqt_plot](http://wiki.ros.org/rqt_plot)
* [PyQtGraph](http://www.pyqtgraph.org/) *(optional but recommended)*



## Compile

```shell
$ catkin_make
```

### Run

```shell
$ source ./devel/setup.bash
$ rosrun rta_nodes plot -loop -echo -rate=100
```

The `loop`, `echo` and `rate` flags can be ommited. Default rate is 1000. The application will prompt you with a `rqt_plot` command which has to be executed in an independent terminal window - the plot will then be populated with data by the chosen topics.

`rqt_plot` is most useful if used with [PyQtGraph](http://www.pyqtgraph.org/) and its auto scaling functionality (buttom on the bottom left of the plot).

![Accelerations visualized using rqt_plot with PyQtGraph](https://cloud.githubusercontent.com/assets/3015996/8634113/a42f4286-27eb-11e5-94ce-5ad71d107bf0.png)
