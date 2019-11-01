# Dualshock4
https://github.com/naoki-mizuno/ds4_driver

```
git clone https://github.com/naoki-mizuno/ds4drv --branch devel
cd ds4drv
python2 setup.py install --prefix ~/.local
sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

# environment
```
export ROS_IP=`hostname -I`
export ROS_MASTER_URI=http://xxx.xxx.xxx.xxx:11311
```

# packages
```
sudo apt install -y \
ros-melodic-ackermann-drive \
ros-melodic-nmea-navsat-driver \
python-pyproj \

```
```
sudo pip install Adafruit-BNO055
```

# logging
```
rosbag record /ackmn_drive /fix /nmea_sentence /tf /tf_static
```

# gpsd_client

```
gpsd -S 4000 /dev/serial0
rosrun gpsd_client gpsd_client _host:=localhost _port:=4000
```
