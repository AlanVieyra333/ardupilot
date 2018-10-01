# specify the linux base image with your desired version ubuntu:<version>
FROM ubuntu:14.04

# Docker folder to work.
WORKDIR /root/app

ENV USER=root

COPY Tools/scripts/install-prereqs-ubuntu.sh .

# Initial internal configuration.
RUN apt-get -y update
RUN apt-get -y install sudo iputils-ping lsb-core software-properties-common python-dev python-pip
#RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y
RUN apt-get -y update
RUN apt-get -y install build-essential software-properties-common
RUN pip install -i https://pypi.python.org/simple/ --upgrade pip==9.0
RUN pip install --upgrade --force-reinstall pyserial
RUN pip install future lxml && pip install pymavlink MAVProxy

# Prereqs for build Ardupilot.
RUN ./install-prereqs-ubuntu.sh -y

RUN echo "export PATH=/usr/lib/ccache:$PATH:/root/app/Developer/Drone/Tools/autotest" >>  .bashrc
RUN . ~/.bashrc

#CMD bash
CMD cd ~/app/Developer/Drone && git submodule update --init --recursive && ./waf configure --board bebop --static && ./waf copter
