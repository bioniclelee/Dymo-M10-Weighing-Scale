# Dymo-M10-Weighing-Scale

As a USB device, the Dymo M10 weighing scale requires special permissions for access, if not it will throw the following permissions error:
```
usb.core.USBError: [Errno 13] Access denied (insufficient permissions)
```

Without touching udev rules (and mainly for quick developmental work), a quick workaround would be to build and rosrun the nodes as a root user via:
```
sudo su
```

However, to ensure long-term, "proper" USB permissions, the way to go about it would be through [adding a udev rule](https://stackoverflow.com/questions/3738173/why-does-pyusb-libusb-require-root-sudo-permissions-on-linux) as well as a group that the user can be added to. You can do this by creating a file in ```/etc/udev/rules.d/``` for example with the name ```99 - dymo.rules``` or something similar, and it should contain the following line:
```
SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", MODE="0664", GROUP="<GROUP NAME>"
```

After that, the user would need to be [added to this newly created group](https://www.howtogeek.com/50787/add-a-user-to-a-group-or-second-group-on-linux/):
```
sudo usermod -a -G <GROUP NAME> username
sudo udevadm control --reload
sudo udevadm trigger
```

This should add you to the newly created group, before refreshing the udev rules and forcing the udev system to see your changes. Now unplug and replug the device or reboot your machine.

Before building the package, if you are not running on a Jetson or something similar, the GPIO libraries would not be imported correctly and will present an error not unlike this one:
```
IOError: [Errno 2] No such file or directory: '/proc/device-tree/compatible'
```
Thus, you will need to comment out all GPIO related code.

On the side of the Jetson, 
