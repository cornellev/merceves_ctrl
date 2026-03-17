To source your ROS environment, run:

```
./source.sh
source install/setup.bash
```

If you are using a controller, start up the following two ROS nodes as follows:

```
ros2 run joy joy_node
ros2 run teleop joyBroadcast
```

You can ignore that if you are using another form of control

To initiate SPI communications with the car, start the following node:

```
ros2 run spicontrol spicontrol
```

To interact with the spicontrol, send an ackermann steering message out with the name "ackDrive".
The spicontrol will listen to this message and use that information to control the car's motion
