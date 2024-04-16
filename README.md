# mqtt_ros_bridge

[![Continuous Integration](https://github.com/benjaminwp18/mqtt_ros_bridge/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/benjaminwp18/mqtt_ros_bridge/actions/workflows/industrial_ci_action.yml)

A ROS2 package for bridging between MQTT and ROS networks

## Installation
### Operating System
ROS2 is pretty tightly coupled to Ubuntu versions. `mqtt_ros_bridge` supports ROS2 Iron Irwini and Humble Hawksbill. Pick a [version](https://docs.ros.org/en/rolling/Releases.html) that works with your OS.

### MQTT
Install an MQTT broker like [Mosquitto](https://mosquitto.org/).

```bash
sudo apt-get update
sudo apt-get install mosquitto mosquitto-clients -y
```

Your broker should now be running:
```bash
you@ubuntu:~ $ service mosquitto status
● mosquitto.service - Mosquitto MQTT v3.1/v3.1.1 Broker
   Loaded: loaded (/lib/systemd/system/mosquitto.service; enabled; vendor preset
   Active: active (running) since Sat 2021-01-30 16:46:20 GMT; 1min 23s ago
     Docs: man:mosquitto.conf(5)
           man:mosquitto(8)
 Main PID: 5390 (mosquitto)
    Tasks: 1 (limit: 2063)
   CGroup: /system.slice/mosquitto.service
           └─5390 /usr/sbin/mosquitto -c /etc/mosquitto/mosquitto.conf
```

## TODO
 - Set QOS
 - Load topic config from file
 - How to handle duplex topics?
 - Services.
 - Dynamic topic advertising
 - Actions.
 - TopicInfo Hashable Key
 - Import MsgLike properly
 - Connection success message
