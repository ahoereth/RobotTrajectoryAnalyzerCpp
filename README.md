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

## Run

Requires `mongod` and `roscore` running.

```shell
$ source ./devel/setup.bash
$ rosrun rta_nodes plot -loop -echo -rate=100
```

The `loop`, `echo` and `rate` flags can be ommited. Default rate is 1000. The application will prompt you with a `rqt_plot` command which has to be executed in an independent terminal window - the plot will then be populated with data by the chosen topics.

`rqt_plot` is most useful if used with [PyQtGraph](http://www.pyqtgraph.org/) and its auto scaling functionality (buttom on the bottom left of the plot).


## Example Runs
### Acceleration
*All joints, `dummy1` dataset.*

![Acceleration data in dummy1 of all joints](https://cloud.githubusercontent.com/assets/3015996/8634113/a42f4286-27eb-11e5-94ce-5ad71d107bf0.png)

### Position + Velocity
*Left shoulder lift joint, `dummy1` dataset.*

On the left you see an example terminal output: First the application asks for a list of joints to plot and then for a view to plot. Given this information a rqt_plot command is generated before the publishing is initiated.

![Position + Velocity of the left shoulder lift joint data in dummy1]( https://cloud.githubusercontent.com/assets/3015996/9228549/4d488170-4119-11e5-8ae3-035e0eb7f7bd.png)

### Position + Velocity + Movement
*Left shoulder lift joint, `dummy1` dataset.*

The graph shows an obvious limitation of `rqt_plot`: Teal and pink describe different movement directions which are only relevant in specific parts of the plot but plot as 0 values when not relevant. Therefore the plot looks rather noisy.

![Position + Velocity + MovementDirection](https://cloud.githubusercontent.com/assets/3015996/9230927/8d6ddf78-4125-11e5-9d59-5d5e0186bcda.png)
