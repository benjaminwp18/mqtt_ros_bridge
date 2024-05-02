# mqtt_ros_bridge

[![Continuous Integration](https://github.com/benjaminwp18/mqtt_ros_bridge/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/benjaminwp18/mqtt_ros_bridge/actions/workflows/industrial_ci_action.yml)

A ROS2 package for bridging between MQTT and ROS networks

## Installation
### Operating System and ROS2
ROS2 is pretty tightly coupled to Ubuntu versions. `mqtt_ros_bridge` supports ROS2 Iron Irwini. Use [this guide](https://docs.ros.org/en/iron/Installation.html) to pick an OS and install Iron.

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

### Setting up a Workspace
Create a ROS workspace and clone the repo.

```bash
mkdir -p workspace/src
cd workspace/src
git clone https://github.com/benjaminwp18/mqtt_ros_bridge.git
```

### Running the Bridge
Build and source the package from your workspace (`workspace`, not `workspace/src`). You may see `EasyInstallDeprecationWarning: easy_install command is deprecated`, but that can be ignored as long as the package doesn't say "failed".

```bash
colcon build --symlink-install
. install/setup.sh
```

Now start the demo by running these commands in two separate terminals:
```bash
ros2 launch mqtt_ros_bridge demo_pub_launch.py
```
```bash
ros2 launch mqtt_ros_bridge demo_sub_launch.py
```

Or launch the bridge in your own launch file with:
```python
config = os.path.join(
   get_package_share_directory('mqtt_ros_bridge'),
   'config',
   'your_config_file.yaml'
)
```


## TODO
 - ROS services
 - ROS actions
 - Dynamic topic advertising
