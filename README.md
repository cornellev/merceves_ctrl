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

To initiate USB-serial communications with the car (the Pico boards now talk over their native USB "virtual UART" port instead of SPI), start the following node:

```
ros2 run spicontrol spicontrol
```

By default this expects the motor board on `/dev/ttyACM0` and the RPM board on `/dev/ttyACM1`. If your boards enumerate on different ports (check with `ls /dev/ttyACM*`), either edit `src/spi_crtl/config/spicontrol_params.yaml` or launch with the params file instead of running the executable directly:

```
ros2 launch spicontrol launch.py
```

**To interact with the spicontrol, send an ackermann steering message out with the name "ackDrive".**
The spicontrol will listen to this message and use that information to control the car's motion
