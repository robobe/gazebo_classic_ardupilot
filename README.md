# Gazebo classic (ver 11) Ardupilot plugin

- Ardupilot plugin
- Gimbal (servo like) plugin

# Gimbal
## Gimbal Ardupilot connect
Ardupilot plugin defined channels that recived pwm from SITL
Each channel connect to SDF joint and can applay force, velocity or effort via PID control
We add FORWARD option that map the channel data to gazebo `Request` message and publish the message to the gimbal plugin

[Gazebo request](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/msgs/request.proto)

```xml title="Ardpilot control"
 <control channel="8">
    <type>FORWARD</type>
    <offset>1</offset>
    <p_gain>0.20</p_gain>
    <i_gain>0</i_gain>
    <d_gain>0</d_gain>
    <i_max>0</i_max>
    <i_min>0</i_min>
    <cmd_max>2.5</cmd_max>
    <cmd_min>-2.5</cmd_min>
    <jointName>gimbal_small_2d::tilt_joint</jointName>
    <multiplier>1000</multiplier>
    <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
</control>
```

|  channel |  Request |
| -------- | -------- |
| channel  | id        |
| jointName| request    |
|      | data (fix value)  |
| dbl_data | channel command |


!!! note ""
    For know the plugin not use the the id (channel) data
     

## Gimbal gazebo message

The gimbal expose `command_request` topic that recived gazebo `Request` message

```bash title="use command_request from CLI"
gz topic -p "/gazebo/default/iris_demo/command_request"  \
"gazebo.msgs.Request" -m 'id: 1, request: "pitch", data: "pwm", dbl_data: 1100'

gz topic -p "/gazebo/default/iris_demo/command_request"  \
"gazebo.msgs.Request" -m 'id: 1, request: "pitch", data: "pwm", dbl_data: 1900'
```