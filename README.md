# Task Assignment and route-planning among multiple autonomous vehicles system

## 1. Development Environment

* OS: Ubuntu 16.04, Raspbian 
* Compiler: gcc 4.7+
* Building System: CMake

## 2. Install Dependencies

# Update system
```
$ sudo apt-get update
$ sudo apt-get -y upgrade
```

# Development tools for C++, java, and python 
```
$ sudo apt-get -y install build-essential git cmake
$ sudo apt-get -y install openjdk-8-jdk
$ sudo apt-get -y install python-dev python-pip
$ sudo apt-get -y install python-numpy python-scipy python-matplotlib
```

# Commonly used libraries 
```
$ sudo apt-get install autoconf
$ sudo apt-get install libglib2.0-dev
$ sudo apt-get -y install autotools-dev automake autopoint libtool
$ sudo apt-get -y install libopencv-dev python-opencv
$ sudo apt-get -y install libboost-all-dev libeigen3-dev
$ sudo apt-get -y install libcgal-dev
```

# Install Visual Studio code (Optional)
Can downloaded from https://code.visualstudio.com/ and installed mannually. Otherwise, run:
```
$ sudo apt-get -y install apt-transport-https 
$ curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
$ sudo mv microsoft.gpg /etc/apt/trusted.gpg.d/microsoft.gpg
$ sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
$ sudo apt-get update
$ sudo apt-get -y install code
```

# LCM (Lightweight Communications and Marshalling)
Check https://lcm-proj.github.io/index.html for more information about LCM
```
Create a folder
$ mkdir -p ~/software/lcm-lib
$ cd ~/software/lcm-lib

Clone LCM source, compile and install
$ git clone https://github.com/lcm-proj/lcm.git lcm
$ cd lcm
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install

Post install
$ export LCM_INSTALL_DIR=/usr/local/lib
$ echo $LCM_INSTALL_DIR > /etc/ld.so.conf.d/lcm.conf
```

# Spot (Required by LTL)
Download the latest version of spot from website: https://spot.lrde.epita.fr/install.html
Extract files into ~/software
```
$ cd ~/software
$ ./configure
$ ./configure --disable-python
$ make
$ sudo make install

$ sudo apt-get install gedit
$ gedit ~/.bashrc
Add the following line to your ~/.bashrc
$ export LD_LIBRARY_PATH=:/usr/local/lib/
```


## 3. Set up workspace
Set up the workspace at any location as you prefer. Here I use "~/Workspace/cbba_sim" as example
```
$ mkdir -p ~/Workspace/cbba_sim
$ cd ~/Workspace/cbba_sim
$ git init
$ git remote add origin https://github.com/jfangwpi/task_allocation.git
$ git pull origin master
```

## 4. Build the project
```
$ mdkir build
$ cd build
$ cmake ../src
$ make
```

## 5. Set LCM path (Not required right now)
```
$ gedit ~/.bashrc
Add the follwing lines to the ~/.bashrc
$ export LTLSAMPLING=$HOME/Workspace/cbba_sim
$ export PYTHONPATH=$LTLSAMPLING/src/lcmtypes/python:$LTLSAMPLING/src/lcmtypes/python/communicate_data:$PYTHONPATH
```

## 6. Test the example
### Communicate among raspberry pi's
Open 3 terminals, in each terminal type:
```
$ cd ~/Workspace/cbba_sim/build/bin
```
In terminal 1, type:
```
$ ./test_pi1
```
Similar to terminal 2 and terminal 3.
Totally 4 iterations are required before convergence. The process of CBBA among 3 pi's is shown as

<img src="/data/iter1.png" align="middle" height="500" >
<img src="/data/iter2.png" align="middle" height="500" >
<img src="/data/iter3.png" align="middle" height="500" >
<img src="/data/iter4.png" align="middle" height="500" >





