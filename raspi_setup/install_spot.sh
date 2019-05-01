#!/bin/bash

# You must extract the spot folder to ~/Software first. This program installs spot
# Link for spot: https://spot.lrde.epita.fr/install.html
cd ~/Software/spot-2.7
sh ./configure --enable-rdm-tests CXXFLAGS='-ftrack-macro-expansion=0'
make -j2
sudo make install

