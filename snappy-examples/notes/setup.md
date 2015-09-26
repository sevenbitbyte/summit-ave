Snappy Setup
============

Derived from Martin Pitt's ["Snappy package for ROS tutorial"](http://www.piware.de/2015/01/snappy-package-for-robot-operating-system-tutorial/)

## Install Ubuntu Dev Tools

https://wiki.ubuntu.com/SimpleSbuild

```
sudo apt-get install sbuild debhelper ubuntu-dev-tools
sudo adduser $USER sbuild
mkdir -p $HOME/ubuntu/scratch
export SBUILD_FSTAB=/etc/schroot/sbuild/fstab
echo "/home/$USER/ubuntu/scratch  /scratch          none  rw,bind  0  0" >> $SBUILD_FSTAB
echo "/home/$USER                 /home/$USER  none  rw,bind  0  0" >> $SBUILD_FSTAB
```

```
cp default.sbuildrc ~/.sbuildrc
mkdir -p $HOME/ubuntu/logs
sg sbuild
sbuild-update --keygen
mk-sbuild vivid
```

```
chroot -c vivid-amd64 -u root
apt-get install sudo
```

## Build ROS in schroot


```
bzr branch lp:~snappy-dev/snappy-hub/ros-tutorialsbzr branch lp:~snappy-dev/snappy-hub/ros-tutorials

```
