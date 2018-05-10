# fencing_referee_bot
turtlebot epee referee

How to run:

start turtlebot base:

```
roslaunch tbot2_launch tbot2.launch
```

start color filters:

```
rosrun fencing_referee filter_red
```

```
rosrun fencing_referee filter_green
```

start score tracker:

```
rosrun fencing_referee scorelight_detector
```

start tournament tracker:

```
rosrun fencing_referee tournament
```

start white filter and camera rotate:

```
rosrun fencing_referee filter_white
```

```
rosrun fencing_referee camera_rotate
```
