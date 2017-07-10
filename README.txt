tfischer 23/11/16

  I had to add udev rules to be able to read hid devices as non root.
  Added the file /etc/udev/rules.d/99-hidraw-permissions.rules with
  the following content:

    KERNEL=="hidraw*", SUBSYSTEM=="hidraw", MODE="0664", GROUP="uucp"

  Change the group name to one that fits your system. The user running
  the program should be part of that group. Update your udev rules with

    sudo udevadm trigger

  Connect your device and make sure it loads with this permissions:

    ls -lh /dev/hidraw0

  or whatever your device is called. It can be checked after plugging
  in using 'dmesg'.
