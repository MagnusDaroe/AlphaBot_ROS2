# AlphaBot_ROS2


## Create the gpio group if it doesn't exist
sudo groupadd gpio

## Add your user to the gpio group
sudo usermod -aG gpio $USER

## Give gpio group access to /dev/gpiomem
sudo chown root:gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem

## Log out and log back in to apply the group changes
logout



# Camera
sudo apt update
sudo apt install ros-humble-cv-bridge
pip3 install opencv-python-headless



sudo apt install libraspberrypi-bin v4l-utils ros-humble-v4l2-camera
sudo apt install ros-humble-image-transport-plugins
sudo usermod -aG video dar


sudo nano /etc/udev/rules.d/99-camera.rules
KERNEL=="video[0-9]*", GROUP="video", MODE="0660"
sudo groupadd video
sudo usermod -aG video username
sudo udevadm control --reload-rules
sudo udevadm trigger
