#!/usr/bin/env bash
#
# Might help when, despite installing Nvidia Container Toolkit, necessary libraries cannot be found
#
# Ref: https://github.com/NVIDIA/nvidia-container-toolkit/issues/154#issuecomment-1817890699

sudo apt-get update
sudo apt install apt-transport-https ca-certificates curl software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu focal stable"
apt-cache policy docker-ce
sudo apt install docker-ce
