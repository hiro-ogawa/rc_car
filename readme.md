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

