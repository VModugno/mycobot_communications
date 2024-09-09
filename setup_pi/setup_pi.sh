#!/usr/bin/env bash

set -eux -o pipefail

WLAN_PASS=$1

sudo apt update

curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh ./get-docker.sh
sudo groupadd docker
sudo usermod -aG docker $USER

sudo apt install network-manager -y
sudo bash -c "echo 'network: {config: disabled}' > /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg"
cat 10-my-network-config.yaml | sed "s/REPLACE_PASSWORD/${WLAN_PASS}/g" > /etc/netplan/10-my-config.yaml
sudo netplan generate
sudo netplan apply