sudo apt-get install ros-$ROS_DISTRO-image-pipeline
sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d
sudo udevadm control --reload-rules && sudo udevadm trigger