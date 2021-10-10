# sawMaxonEPOS

This SAW component contains code for interfacing with Maxon EPOS controllers.

The `ros` folder contains code for a ROS node that interfaces with the sawMaxonEPOS component.

If needed, one can also add OpenIGTLink support using sawOpenIGTLink (contact the sawMaxonEPOS developers if you need help with this).

# Links
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
 * cisst libraries: https://github.com/jhu-cisst/cisst
 * Qt for user interface
 * ROS (optional)

# Build

You can find some documentation re. compiling cisst and SAW components in the [dVRK wiki](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild#catkin-build-and-rosinstall)(best source if you're using Linux with ROS) and the [cisst wiki](https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake)(more details and provides instructions for Windows as well).

For Linux with ROS, we provide a rosinstall file to retrieve all the git repositories you need for sawMaxonEPOS:
```
wstool merge https://raw.githubusercontent.com/jhu-saw/sawMaxonEPOS/devel/ros/ndi_tracker.rosinstall
```

# Running the examples

## Linux permissions

NDI trackers use a serial port to communicate.  When connecting your tracker to your computer, a pseudo device will be added to the `/dev` directory.   Usually something like `/dev/ttyS01`, `/dev/ttyUSB0` or `/dev/ttyACM0`.  Using the command `dmesg` can help identify which device is used.  Check the file permissions on said device, e.g.,
```sh
ls -al /dev/ttyUSB0
crw-rw---- 1 root dialout 188, 0 Jan  3 09:32 /dev/ttyUSB0
```
On Ubuntu, the OS usually sets the ownership of `/dev/ttyUSB0` to `root` and the group to `dialout`.   To grant permissions to read and write to the device, use the command `sudo adduser <user_id> dialout` to add users to the `dialout` group.   Please note that the user has to logout/login for the new group membership to take effect.
