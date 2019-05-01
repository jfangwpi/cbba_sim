#!/bin/bash


# This section reinstalls the java SDK. Without this installing from devel-test_setup would not work
cd

sudo mv /usr/lib/jvm/java-8-openjdk-armhf/jre/lib/arm/client /usr/lib/jvm/java-8-openjdk-armhf/jre/lib/arm/server
sudo apt-get purge openjdk-8-jre-headless -y
sudo apt-get install openjdk-8-jre-headless -y
sudo apt-get install openjdk-8-jre -y
sudo mv /usr/lib/jvm/java-8-openjdk-armhf/jre/lib/arm/client /usr/lib/jvm/java-8-openjdk-armhf/jre/lib/arm/server

# Runs the devel-test_setup scripts. Choose which one you want to install here
cd /media/pi/"NEW VOLUME"/devel_setup-master
sudo chmod +x install_sbc.bash
sh ./install_sbc.bash
cd
