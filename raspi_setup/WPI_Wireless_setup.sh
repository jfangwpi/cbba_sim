#!/bin/bash

sudo mkdir -p /etc/pki/tls/certs
sudo mv /home/pi/Downloads/* /etc/pki/tls/certs/
sudo chown root:root /etc/pki/tls/certs/*
sudo chmod 600 /etc/pki/tls/certs/*

cd /etc/pki/tls/certs
sudo openssl pkcs12 -in certificate.p12 -out temp.pem -passout pass:TEMP
sudo openssl pkcs12 -export -in temp.pem -out certificate.p12 -passin pass:TEMP

cd /etc/pki/tls/certs
sudo rm WPI-Wireless-TEMP.p12 temp.pem

sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
