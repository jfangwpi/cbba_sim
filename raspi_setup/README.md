Enter these lines of code into the terminal:

cd /media/pi/"NEW VOLUME"
sudo chmod +x fix_java_and_devel_setup.sh
sudo chmod +x install_spot.sh

when running the sh files type "sh FILENAME.sh" to run


________________ For WPI-Wireless Setup___________
network={
        ssid="WPI-Wireless"
        key_mgmt=WPA-EAP
        proto=WPA2
        pairwise=CCMP
        group=CCMP
        eap=TLS
        identity="jroneill@wpi.edu"
        ca_cert=""
        private_key="/etc/pki/tls/certs/certificate.p12"
        private_key_passwd="pi2password"
}
