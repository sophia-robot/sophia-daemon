# Sophia Robot Daemon

# Install
To install run the following command 
```
$ sudo ./install.sh
```

# Operation
## Run
To run the daemon run the following command from any terminal
```
$ sophia start robot
```

## Stop
To stop the daemon run the following command from any terminal
```
$ sophia stop robot
```

## Run in Sophia workspace
```
$ sophia run foo.py
```

# ROS 2 Topics
## Feedforward
Topic:
```
/ref/pos
```
Type:
```
JointState [sensor_msgs.msg]
```

## Feedback

### Position
Topic:
```
/state/pos
```
Type:
```
JointState [sensor_msgs.msg]
```

### Torque
Topic:
```
/state/torque
```
Type:
```
JointState [sensor_msgs.msg]
```

### Accelerometer
Topic:
```
/state/acc
```
Type:
```
Twist [geometry_msgs.msg]
```

