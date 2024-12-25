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
